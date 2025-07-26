import sys
import trimesh

def convert_obj_to_stl(input_path, output_path=None):
    mesh = trimesh.load(input_path, force='mesh')

    if output_path is None:
        output_path = input_path.replace('.obj', '.stl')

    mesh.export(output_path)
    print(f"Converted: {input_path} → {output_path}")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python obj_to_stl.py input.obj [output.stl]")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None
    convert_obj_to_stl(input_file, output_file)