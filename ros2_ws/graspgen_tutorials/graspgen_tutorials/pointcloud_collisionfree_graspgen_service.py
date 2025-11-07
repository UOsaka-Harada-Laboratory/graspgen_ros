import json
import torch
import numpy as np

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import trimesh.transformations as tra

from grasp_gen.grasp_server import GraspGenSampler, load_grasp_cfg
from grasp_gen.dataset.eval_utils import save_to_isaac_grasp_format
from grasp_gen.utils.meshcat_utils import (
    create_visualizer,
    get_color_from_score,
    visualize_grasp,
    visualize_pointcloud,
)
from grasp_gen.utils.point_cloud_utils import point_cloud_outlier_removal, filter_colliding_grasps
from grasp_gen.robot import get_gripper_info


class GraspGenServer(Node):
    def __init__(self):
        super().__init__('graspgen_server')

        self.declare_parameter("object_pointcloud_path", "/share/pcjson/example.json")
        self.declare_parameter("scene_pointcloud_path", "/share/pcjson/table.json")
        self.declare_parameter("gripper_name", "")
        self.declare_parameter("gripper_config_path", "")
        self.declare_parameter("gripper_mesh_path", "")
        self.declare_parameter("save_results", True)
        self.declare_parameter("grasp_result_path", "/tmp/grasp_result.yaml")
        self.declare_parameter("grasp_threshold", -1.0)
        self.declare_parameter("num_grasps", 500)
        self.declare_parameter("return_topk", True)
        self.declare_parameter("topk_num_grasps", 10)
        self.declare_parameter("collision_threshold", 0.02)  # 0.02 = 2mm
        self.declare_parameter("max_scene_points", 8192)
        self.declare_parameter("no_visualization", False)

        self.object_pointcloud_path = self.get_parameter("object_pointcloud_path").get_parameter_value().string_value
        self.scene_pointcloud_path = self.get_parameter("scene_pointcloud_path").get_parameter_value().string_value
        self.gripper_name = self.get_parameter("gripper_name").get_parameter_value().string_value
        self.gripper_config_path = self.get_parameter("gripper_config_path").get_parameter_value().string_value
        self.gripper_mesh_path = self.get_parameter("gripper_mesh_path").get_parameter_value().string_value
        self.save_results = self.get_parameter("save_results").get_parameter_value().bool_value
        self.output_file = self.get_parameter("grasp_result_path").get_parameter_value().string_value
        self.grasp_threshold = self.get_parameter("grasp_threshold").get_parameter_value().double_value
        self.num_grasps = self.get_parameter("num_grasps").get_parameter_value().integer_value
        self.return_topk = self.get_parameter("return_topk").get_parameter_value().bool_value
        self.topk_num_grasps = self.get_parameter("topk_num_grasps").get_parameter_value().integer_value
        self.collision_threshold = self.get_parameter("collision_threshold").get_parameter_value().double_value
        self.max_scene_points = self.get_parameter("max_scene_points").get_parameter_value().integer_value
        self.no_visualization = self.get_parameter("no_visualization").get_parameter_value().bool_value

        self.marker_pub = self.create_publisher(MarkerArray, "grasp_markers", 1)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.graspgen_service = self.create_service(Empty, "generate_grasp", self.generate_grasp_callback)
        self.get_logger().info("Grasp generation service '/generate_grasp' ready.")

    def process_point_cloud(self, pc, grasps, grasp_conf, pc_colors=None):
        """Process point cloud and grasps by centering them."""
        scores = get_color_from_score(grasp_conf, use_255_scale=True)
        self.get_logger().info(f"Scores with min {grasp_conf.min():.3f} and max {grasp_conf.max():.3f}")

        # Ensure grasps have correct homogeneous coordinate
        grasps[:, 3, 3] = 1

        # Center point cloud and grasps
        T_subtract_pc_mean = tra.translation_matrix(-pc.mean(axis=0))
        pc_centered = tra.transform_points(pc, T_subtract_pc_mean)
        grasps_centered = np.array(
            [T_subtract_pc_mean @ np.array(g) for g in grasps.tolist()]
        )

        # Add red tint to colors if RGB data is available
        pc_colors_centered = pc_colors
        if pc_colors is not None:
            pc_colors_centered = pc_colors.copy().astype(np.float32)
            # Add red tint: increase red channel by 40% while keeping it in valid range
            pc_colors_centered[:, 0] = np.clip(pc_colors_centered[:, 0] * 1.4, 0, 255)
            pc_colors_centered = pc_colors_centered.astype(np.uint8)

        return pc_centered, grasps_centered, scores, T_subtract_pc_mean, pc_colors_centered

    def generate_grasp_callback(self, request, response):
        self.get_logger().info(f"Running grasp generator on: {self.object_pointcloud_path} with {self.scene_pointcloud_path}")

        vis = None if self.no_visualization else create_visualizer()
        grasp_cfg = load_grasp_cfg(self.gripper_config_path)
        gripper_name = grasp_cfg.data.gripper_name
        gripper_info = get_gripper_info(gripper_name)
        gripper_collision_mesh = gripper_info.collision_mesh

        self.get_logger().info(f"Using gripper: {gripper_name}")
        self.get_logger().info(
            f"Gripper collision mesh has {len(gripper_collision_mesh.vertices)} vertices"
        )

        grasp_sampler = GraspGenSampler(grasp_cfg)
        object_pc_data = json.load(open(self.object_pointcloud_path, "rb"))
        pc = np.array(object_pc_data["pc"])
        pc_color = np.array(object_pc_data["pc_color"])
        grasps = np.array(object_pc_data["grasp_poses"])
        grasp_conf = np.array(object_pc_data["grasp_conf"])

        scene_pc_data = json.load(open(self.scene_pointcloud_path, "rb"))
        scene_pc = np.array(scene_pc_data["pc"])
        scene_pc_color = np.array(scene_pc_data["pc_color"])

        object_pc_torch = torch.from_numpy(pc.astype(np.float32))
        pc_filtered, pc_removed = point_cloud_outlier_removal(object_pc_torch)
        pc_filtered = pc_filtered.numpy()
        pc_removed = pc_removed.numpy()
        self.get_logger().info(
            f"Filtered object point cloud: {len(pc_filtered)} outlier points (removed {len(pc_removed)})"
        )

        grasps_inferred, grasp_conf_inferred = GraspGenSampler.run_inference(
            pc_filtered,
            grasp_sampler,
            grasp_threshold=self.grasp_threshold,
            num_grasps=self.num_grasps,
            topk_num_grasps=self.topk_num_grasps,
        )

        if len(grasps_inferred) > 0:
            grasp_conf_inferred = grasp_conf_inferred.cpu().numpy()
            grasps_inferred = grasps_inferred.cpu().numpy()
            grasps_inferred[:, 3, 3] = 1

            if "suction" in self.gripper_name.lower():
                grasps_inferred = np.array([g @ tra.rotation_matrix(np.pi, [1, 0, 0]) for g in grasps_inferred])

            scores_inferred = get_color_from_score(
                grasp_conf_inferred, use_255_scale=True
            )
            self.get_logger().info(
                f"Inferred {len(grasps_inferred)} grasps, with scores ranging from {grasp_conf_inferred.min():.3f} - {grasp_conf_inferred.max():.3f}"
            )

            pc_centered, grasps_centered, scores, T_center, object_colors_centered = (
                self.process_point_cloud(
                    pc_filtered, grasps_inferred, grasp_conf_inferred, pc_color
                )
            )
            scene_pc_centered = tra.transform_points(scene_pc, T_center)

            # Add red tint to scene colors if RGB data is available
            scene_colors_centered = scene_pc_color
            if scene_pc_color is not None:
                scene_colors_centered = scene_pc_color.copy().astype(np.float32)
                # Add red tint: increase red channel by 40% while keeping it in valid range
                scene_colors_centered[:, 0] = np.clip(scene_colors_centered[:, 0] * 1.4, 0, 255)
                scene_colors_centered = scene_colors_centered.astype(np.uint8)

            # Downsample scene point cloud for faster collision checking (keep full resolution for visualization)
            if len(scene_pc_centered) > self.max_scene_points:
                indices = np.random.choice(
                    len(scene_pc_centered), self.max_scene_points, replace=False
                )
                scene_pc_downsampled = scene_pc_centered[indices]
                self.get_logger().info(
                    f"Downsampled scene point cloud from {len(scene_pc_centered)} to {len(scene_pc_downsampled)} points for collision checking"
                )
            else:
                scene_pc_downsampled = scene_pc_centered
                self.get_logger().info(
                    f"Scene point cloud has {len(scene_pc_centered)} points (no downsampling needed)"
                )

            # Filter collision grasps using downsampled scene
            collision_free_mask = filter_colliding_grasps(
                scene_pc=scene_pc_downsampled,
                grasp_poses=grasps_centered,
                gripper_collision_mesh=gripper_collision_mesh,
                collision_threshold=self.collision_threshold,
            )

            # Filter grasps to only collision-free ones
            collision_free_grasps = grasps_centered[collision_free_mask]
            collision_free_scores = grasp_conf_inferred[collision_free_mask]
            collision_free_colors = scores[collision_free_mask]

            self.get_logger().info(
                f"Final result: {len(collision_free_grasps)} collision-free grasps out of {len(grasps_inferred)} total grasps"
            )

            # Visualize scene point cloud - use RGB colors if available, otherwise use gray
            if scene_colors_centered is not None:
                visualize_pointcloud(
                    vis, "scene_pc", scene_pc_centered, scene_colors_centered, size=0.002
                )
            else:
                visualize_pointcloud(
                    vis, "scene_pc", scene_pc_centered, [128, 128, 128], size=0.002
                )

            # Visualize object point cloud - use RGB colors if available, otherwise use green
            if object_colors_centered is not None:
                visualize_pointcloud(
                    vis, "object_pc", pc_centered, object_colors_centered, size=0.0025
                )
            else:
                visualize_pointcloud(
                    vis, "object_pc", pc_centered, [0, 255, 0], size=0.0025
                )

            # Visualize collision-free grasps
            for i, (grasp, score) in enumerate(
                zip(collision_free_grasps, collision_free_colors)
            ):
                visualize_grasp(
                    vis,
                    f"collision_free_grasps/{i:03d}/grasp",
                    grasp,
                    color=score,
                    gripper_name=gripper_name,
                    linewidth=0.8,
                )

            # Visualize colliding grasps in red
            colliding_grasps = grasps_centered[~collision_free_mask]
            for i, grasp in enumerate(
                colliding_grasps[:20]
            ):  # Limit to first 20 for clarity
                visualize_grasp(
                    vis,
                    f"colliding_grasps/{i:03d}/grasp",
                    grasp,
                    color=[255, 0, 0],
                    gripper_name=gripper_name,
                    linewidth=0.4,
                )

            # Publish grasp object, and scene tfs and markers
            self.publish_tf_and_marker(collision_free_grasps, colliding_grasps, pc_centered, object_colors_centered, scene_pc_centered, scene_colors_centered)

            if self.output_file != "":
                self.get_logger().info(f"Saving predicted collision-free grasps to {self.output_file}")
                save_to_isaac_grasp_format(
                    collision_free_grasps, collision_free_scores, self.output_file
                )
            else:
                self.get_logger().info("No output file specified, skipping grasp saving")

        else:
            self.get_logger().info("No grasps found from inference!")

        return response

    def publish_tf_and_marker(self, grasps, colliding_grasps, pc_centered, pc_color, scene_pc_centered, scene_color):
        tf_list = []
        marker_array = MarkerArray()

        # Add object point cloud as SPHERE_LIST marker
        obj_pc_marker = Marker()
        obj_pc_marker.header.frame_id = "world"
        obj_pc_marker.header.stamp = self.get_clock().now().to_msg()
        obj_pc_marker.ns = "object"
        obj_pc_marker.id = 10000
        obj_pc_marker.type = Marker.SPHERE_LIST
        obj_pc_marker.action = Marker.ADD
        obj_pc_marker.scale.x = 0.003
        obj_pc_marker.scale.y = 0.003
        obj_pc_marker.scale.z = 0.003
        obj_pc_marker.color.r = 0.7
        obj_pc_marker.color.g = 0.7
        obj_pc_marker.color.b = 0.7
        obj_pc_marker.color.a = 1.0
        
        for pt in pc_centered:
            p = Point()
            p.x, p.y, p.z = pt.tolist()
            obj_pc_marker.points.append(p)

        marker_array.markers.append(obj_pc_marker)

        # Add scene point cloud as SPHERE_LIST marker
        scene_pc_marker = Marker()
        scene_pc_marker.header.frame_id = "world"
        scene_pc_marker.header.stamp = self.get_clock().now().to_msg()
        scene_pc_marker.ns = "scene"
        scene_pc_marker.id = 10000
        scene_pc_marker.type = Marker.SPHERE_LIST
        scene_pc_marker.action = Marker.ADD
        scene_pc_marker.scale.x = 0.003
        scene_pc_marker.scale.y = 0.003
        scene_pc_marker.scale.z = 0.003
        scene_pc_marker.color.r = 0.5
        scene_pc_marker.color.g = 0.5
        scene_pc_marker.color.b = 0.5
        scene_pc_marker.color.a = 1.0
        
        for pt in scene_pc_centered:
            p = Point()
            p.x, p.y, p.z = pt.tolist()
            scene_pc_marker.points.append(p)

        marker_array.markers.append(scene_pc_marker)

        for i, g in enumerate(grasps):
            trans = g[:3, 3].tolist()
            q = tra.quaternion_from_matrix(g[:3, :3])
            w = float(q[0])
            x, y, z = q[1:].tolist()
            quat = [x, y, z, w]

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "object"
            t.child_frame_id = f"grasp_{i}"
            t.transform.translation.x = trans[0]
            t.transform.translation.y = trans[1]
            t.transform.translation.z = trans[2]
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            tf_list.append(t)

            grasp_marker = Marker()
            grasp_marker.header.frame_id = "object"
            grasp_marker.header.stamp = self.get_clock().now().to_msg()
            grasp_marker.ns = "gripper"
            grasp_marker.id = i
            grasp_marker.type = Marker.MESH_RESOURCE
            grasp_marker.action = Marker.ADD
            grasp_marker.pose.position.x = trans[0]
            grasp_marker.pose.position.y = trans[1]
            grasp_marker.pose.position.z = trans[2]
            grasp_marker.pose.orientation.x = quat[0]
            grasp_marker.pose.orientation.y = quat[1]
            grasp_marker.pose.orientation.z = quat[2]
            grasp_marker.pose.orientation.w = quat[3]
            grasp_marker.scale.x = 1.0
            grasp_marker.scale.y = 1.0
            grasp_marker.scale.z = 1.0
            grasp_marker.color.r = 0.0
            grasp_marker.color.g = 1.0
            grasp_marker.color.b = 0.0
            grasp_marker.color.a = 0.3
            grasp_marker.mesh_resource = 'file://' + self.gripper_mesh_path
            grasp_marker.mesh_use_embedded_materials = False
            marker_array.markers.append(grasp_marker)

        for i, g in enumerate(colliding_grasps):
            trans = g[:3, 3].tolist()
            q = tra.quaternion_from_matrix(g[:3, :3])
            w = float(q[0])
            x, y, z = q[1:].tolist()
            quat = [x, y, z, w]

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "object"
            t.child_frame_id = f"colliding_grasp_{i}"
            t.transform.translation.x = trans[0]
            t.transform.translation.y = trans[1]
            t.transform.translation.z = trans[2]
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            tf_list.append(t)

            grasp_marker = Marker()
            grasp_marker.header.frame_id = "object"
            grasp_marker.header.stamp = self.get_clock().now().to_msg()
            grasp_marker.ns = "colliding_gripper"
            grasp_marker.id = i
            grasp_marker.type = Marker.MESH_RESOURCE
            grasp_marker.action = Marker.ADD
            grasp_marker.pose.position.x = trans[0]
            grasp_marker.pose.position.y = trans[1]
            grasp_marker.pose.position.z = trans[2]
            grasp_marker.pose.orientation.x = quat[0]
            grasp_marker.pose.orientation.y = quat[1]
            grasp_marker.pose.orientation.z = quat[2]
            grasp_marker.pose.orientation.w = quat[3]
            grasp_marker.scale.x = 1.0
            grasp_marker.scale.y = 1.0
            grasp_marker.scale.z = 1.0
            grasp_marker.color.r = 1.0
            grasp_marker.color.g = 0.0
            grasp_marker.color.b = 0.0
            grasp_marker.color.a = 0.3
            grasp_marker.mesh_resource = 'file://' + self.gripper_mesh_path
            grasp_marker.mesh_use_embedded_materials = False
            marker_array.markers.append(grasp_marker)

        self.tf_broadcaster.sendTransform(tf_list)
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f"Published {len(tf_list)} grasps as TF + mesh markers.")


def main(args=None):
    rclpy.init(args=args)
    node = GraspGenServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
