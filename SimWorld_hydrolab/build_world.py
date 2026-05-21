import math
import os
import re
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
WORLD_NAME = "hydrolab_randomized"

LOCAL_BASE_FILE = SCRIPT_DIR / "hydrolab_base.sdf"
LOCAL_RANDOMIZED_FILE = SCRIPT_DIR / "hydrolab_built.sdf"

DEFAULT_PX4_WORLDS_DIR = Path("/home/mateusz/AbsoluteEdge/SImulationENV/PX4-Autopilot/Tools/simulation/gz/worlds")


def create_circular_pool(
    name,
    x,
    y,
    inner_radius=1.0,
    wall_thickness=0.15,
    wall_height=0.6,
    segments=24,
    color=(1, 0.92, 0.2, 0.8),
):
    """Create a simple above-ground circular pool with a 2 m inner radius."""
    floor_thickness = 0.1
    floor_radius = inner_radius + wall_thickness
    wall_radius = inner_radius + (wall_thickness / 2.0)
    wall_width = 2 * wall_radius * math.tan(math.pi / segments)
    wall_center_z = (floor_thickness / 2.0) + (wall_height / 2.0)
    water_z = floor_thickness + 0.01
    water_radius = max(inner_radius - 0.05, 0.1)
    color_rgb = color[:3]
    color_alpha = color[3]
    solid_color_str = " ".join(str(component) for component in (*color_rgb, 1))
    water_color_str = " ".join(str(component) for component in color)

    xml = f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 0 0 0 0</pose>
      <link name="link">
        <collision name="floor_collision">
          <pose>0 0 {floor_thickness / 2.0} 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>{floor_radius}</radius>
              <length>{floor_thickness}</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="floor_visual">
          <pose>0 0 {floor_thickness / 2.0} 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>{floor_radius}</radius>
              <length>{floor_thickness}</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>{solid_color_str}</ambient>
            <diffuse>{solid_color_str}</diffuse>
            <specular>0.12 0.12 0.12 1</specular>
          </material>
        </visual>
        <visual name="water_visual">
          <pose>0 0 {water_z} 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>{water_radius}</radius>
              <length>0.02</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>{water_color_str}</ambient>
            <diffuse>{water_color_str}</diffuse>
            <specular>0.2 0.2 0.2 {color_alpha}</specular>
          </material>
        </visual>
    """

    for index in range(segments):
        angle = index * (2 * math.pi / segments)
        segment_x = wall_radius * math.cos(angle)
        segment_y = wall_radius * math.sin(angle)

        xml += f"""
        <collision name="wall_collision_{index}">
          <pose>{segment_x} {segment_y} {wall_center_z} 0 0 {angle}</pose>
          <geometry>
            <box>
              <size>{wall_thickness} {wall_width} {wall_height}</size>
            </box>
          </geometry>
        </collision>
        <visual name="wall_visual_{index}">
          <pose>{segment_x} {segment_y} {wall_center_z} 0 0 {angle}</pose>
          <geometry>
            <box>
              <size>{wall_thickness} {wall_width} {wall_height}</size>
            </box>
          </geometry>
          <material>
            <ambient>{solid_color_str}</ambient>
            <diffuse>{solid_color_str}</diffuse>
            <specular>0.12 0.12 0.12 1</specular>
          </material>
        </visual>
        """

    xml += """
      </link>
    </model>
    """

    return xml


def create_landing_pad(name, x, y, size_x=10.0, size_y=10.0, height=0.1):
    half_height = height / 2.0

    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <pose>0 0 {half_height} 0 0 0</pose>
          <geometry>
            <box>
              <size>{size_x} {size_y} {height}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <pose>0 0 {half_height} 0 0 0</pose>
          <geometry>
            <box>
              <size>{size_x} {size_y} {height}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.15 0.15 0.15 1</ambient>
            <diffuse>0.15 0.15 0.15 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>
    """


def build_world(base_file, objects_xml):
    content = base_file.read_text(encoding="utf-8-sig")
    if "</world>" not in content:
        raise ValueError(f"Missing closing </world> tag in {base_file}")

    new_content = content.replace("</world>", objects_xml + "\n  </world>", 1)
    return re.sub(r'<world name="[^"]+">', f'<world name="{WORLD_NAME}">', new_content, count=1)


def get_output_files():
    output_files = [LOCAL_RANDOMIZED_FILE]
    px4_worlds_dir = Path(os.environ.get("PX4_WORLDS_DIR", DEFAULT_PX4_WORLDS_DIR))

    if px4_worlds_dir.is_dir():
        output_files.append(px4_worlds_dir / LOCAL_RANDOMIZED_FILE.name)

    return output_files


def main():
    objects_xml = "\n".join(
        [
            create_landing_pad("start_base", 0.0, 0.0),
            create_circular_pool("pool_red", 25.0, 0.0, color=(1.0, 0.15, 0.15, 0.8)),
            create_circular_pool("pool_green", 50.0, 5.0, color=(0.15, 0.8, 0.2, 0.8)),
            create_circular_pool("pool_blue", 75.0, -2.0, color=(0.1, 0.3, 1.0, 0.8)),
            create_circular_pool("pool_yellow", 100.0, 6.0, color=(1.0, 0.9, 0.1, 0.8)),
        ]
    )
    world_content = build_world(LOCAL_BASE_FILE, objects_xml)

    written_files = []
    for output_file in get_output_files():
        output_file.write_text(world_content, encoding="utf-8")
        written_files.append(str(output_file))

    print("World generated successfully:")
    for output_file in written_files:
        print(f" - {output_file}")


if __name__ == "__main__":
    main()
