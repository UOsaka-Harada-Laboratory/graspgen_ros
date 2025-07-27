import argparse
import numpy as np
import trimesh
import json


def convert_stl_to_json(input_path, output_path, num_points=2048, scale=1.0):
    mesh = trimesh.load_mesh(input_path)
    mesh.apply_scale(scale)

    points, _ = trimesh.sample.sample_surface(mesh, num_points)
    points = np.array(points, dtype=np.float32)
    colors = np.ones_like(points) * 255

    data = {
        "pc": points.tolist(),
        "pc_color": colors.tolist(),
        "grasp_poses": [],
        "grasp_conf": []
    }

    with open(output_path, "w") as f:
        json.dump(data, f, indent=2)

    print(f"Saved JSON to {output_path} with {len(points)} points.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert STL to GraspGen-style JSON (with empty grasp fields).")
    parser.add_argument("input_stl", type=str, help="Input STL file path")
    parser.add_argument("output_json", type=str, help="Output JSON file path")
    parser.add_argument("--num_points", type=int, default=2048, help="Number of sampled points")
    parser.add_argument("--scale", type=float, default=1.0, help="Scaling factor for mesh")

    args = parser.parse_args()
    convert_stl_to_json(args.input_stl, args.output_json, args.num_points, args.scale)
