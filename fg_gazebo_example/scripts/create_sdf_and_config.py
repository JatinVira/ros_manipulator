import os

colors = {
    "yellow": "1 1 0 1",
    "green": "0 1 0 1",
    "gray": "0.5 0.5 0.5 1",
    "peach": "1 0.8 0.6 1",
    "pink": "1 0.4 0.7 1",
    "blue": "0 0 1 1"
}

models_dir = os.path.join(os.getcwd(), 'models')
os.makedirs(models_dir, exist_ok=True)

for color, rgba in colors.items():
    color_dir = os.path.join(models_dir, color)
    os.makedirs(color_dir, exist_ok=True)

    # Create config file
    config_content = f'''<model>
  <name>{color}_cube</name>
  <version>1.0</version>
  <sdf>{color}_cube.sdf</sdf>
  <author>
    <name>Jatin Vira</name>
    <email>jvira78@gmail.com</email>
  </author>
  <description>
    A {color} cube model
  </description>
</model>
'''
    config_path = os.path.join(color_dir, f'{color}_cube.config')
    with open(config_path, 'w') as config_file:
        config_file.write(config_content)

    # Create SDF file
    sdf_content = f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{color}_cube">
    <link name="cube_link">
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.000025</ixx>
          <iyy>0.000025</iyy>
          <izz>0.000025</izz>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyz>0.0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.03 0.03 0.03</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.03 0.03 0.03</size>
          </box>
        </geometry>
        <material>
          <ambient>{rgba}</ambient>
          <diffuse>{rgba}</diffuse>
          <specular>0.5 0.5 0.5 1</specular>
          <emissive>0 0 0 0</emissive>
        </material>
      </visual>
      <velocity_decay>
        <linear>0.0</linear>
        <angular>0.0</angular>
      </velocity_decay>
      <gravity>1</gravity>
      <self_collide>true</self_collide>
      <kinematic>false</kinematic>
    </link>
  </model>
</sdf>
'''
    sdf_path = os.path.join(color_dir, f'{color}_cube.sdf')
    with open(sdf_path, 'w') as sdf_file:
        sdf_file.write(sdf_content)

print("Config and SDF files created successfully.")
