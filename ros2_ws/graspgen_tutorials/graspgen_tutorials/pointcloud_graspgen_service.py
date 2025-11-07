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
from grasp_gen.utils.point_cloud_utils import point_cloud_outlier_removal


class GraspGenServer(Node):
    def __init__(self):
        super().__init__('graspgen_server')

        self.declare_parameter("object_pointcloud_path", "/share/pcjson/example.json")
        self.declare_parameter("gripper_name", "")
        self.declare_parameter("gripper_config_path", "")
        self.declare_parameter("gripper_mesh_path", "")
        self.declare_parameter("save_results", True)
        self.declare_parameter("grasp_result_path", "/tmp/grasp_result.yaml")
        self.declare_parameter("grasp_threshold", -1.0)
        self.declare_parameter("num_grasps", 500)
        self.declare_parameter("return_topk", True)
        self.declare_parameter("topk_num_grasps", 10)
        self.declare_parameter("no_visualization", False)

        self.pointcloud_path = self.get_parameter("object_pointcloud_path").get_parameter_value().string_value
        self.gripper_name = self.get_parameter("gripper_name").get_parameter_value().string_value
        self.gripper_config_path = self.get_parameter("gripper_config_path").get_parameter_value().string_value
        self.gripper_mesh_path = self.get_parameter("gripper_mesh_path").get_parameter_value().string_value
        self.save_results = self.get_parameter("save_results").get_parameter_value().bool_value
        self.output_file = self.get_parameter("grasp_result_path").get_parameter_value().string_value
        self.grasp_threshold = self.get_parameter("grasp_threshold").get_parameter_value().double_value
        self.num_grasps = self.get_parameter("num_grasps").get_parameter_value().integer_value
        self.return_topk = self.get_parameter("return_topk").get_parameter_value().bool_value
        self.topk_num_grasps = self.get_parameter("topk_num_grasps").get_parameter_value().integer_value
        self.no_visualization = self.get_parameter("no_visualization").get_parameter_value().bool_value

        self.marker_pub = self.create_publisher(MarkerArray, "grasp_markers", 1)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.graspgen_service = self.create_service(Empty, "generate_grasp", self.generate_grasp_callback)
        self.get_logger().info("Grasp generation service '/generate_grasp' ready.")

    def process_point_cloud(self, pc, grasps, grasp_conf):
        if len(grasp_conf) == 0:
            self.get_logger().warn("grasp_conf is empty, skipping score range print.")
            scores = []
        else:
            scores = get_color_from_score(grasp_conf, use_255_scale=True)
            self.get_logger().info(f"Scores with min {grasp_conf.min():.3f} and max {grasp_conf.max():.3f}")

        grasps = np.array(grasps)
        if grasps.shape[0] > 0:
            grasps[:, 3, 3] = 1

        T_subtract_pc_mean = tra.translation_matrix(-pc.mean(axis=0))
        pc_centered = tra.transform_points(pc, T_subtract_pc_mean)
        grasps_centered = np.array(
            [T_subtract_pc_mean @ np.array(g) for g in grasps.tolist()]
        ) if grasps.shape[0] > 0 else []

        return pc_centered, grasps_centered, scores

    def generate_grasp_callback(self, request, response):
        self.get_logger().info(f"Running grasp generator on: {self.pointcloud_path}")
        vis = None if self.no_visualization else create_visualizer()
        grasp_cfg = load_grasp_cfg(self.gripper_config_path)
        gripper_name = grasp_cfg.data.gripper_name
        grasp_sampler = GraspGenSampler(grasp_cfg)

        data = json.load(open(self.pointcloud_path, "rb"))
        pc = np.array(data["pc"])
        pc_color = np.array(data["pc_color"])
        grasps = np.array(data["grasp_poses"])
        grasp_conf = np.array(data["grasp_conf"])

        pc_centered, grasps_centered, scores = self.process_point_cloud(
            pc, grasps, grasp_conf
        )

        if not self.no_visualization and pc is not None:
            visualize_pointcloud(vis, "pc", pc_centered, pc_color, size=0.0025)

        pc_filtered, pc_removed = point_cloud_outlier_removal(
            torch.from_numpy(pc_centered)
        )
        pc_filtered = pc_filtered.numpy()
        pc_removed = pc_removed.numpy()
        visualize_pointcloud(vis, "pc_removed", pc_removed, [255, 0, 0], size=0.003)

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

            for j, grasp in enumerate(grasps_inferred):
                visualize_grasp(
                    vis,
                    f"grasps_objectpc_filtered/{j:03d}/grasp",
                    grasp,
                    color=scores_inferred[j],
                    gripper_name=gripper_name,
                    linewidth=0.6,
                )

            self.publish_tf_and_marker(grasps_inferred, grasp_conf_inferred, pc_centered, pc_color)

            if self.output_file != "":
                self.get_logger().info(f"Saving predicted grasps to {self.output_file}")
                save_to_isaac_grasp_format(
                    grasps_inferred, grasp_conf_inferred, self.output_file
                )
            else:
                self.get_logger().info("No output file specified, skipping grasp saving")

        else:
            self.get_logger().info("No grasps found from inference!")

        return response

    def publish_tf_and_marker(self, grasps, scores, pc_centered, pc_color):
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

        for i, (g, s) in enumerate(zip(grasps, scores)):
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

        self.tf_broadcaster.sendTransform(tf_list)
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f"Published {len(tf_list)} grasps as TF + mesh markers.")


def main(args=None):
    rclpy.init(args=args)
    node = GraspGenServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
