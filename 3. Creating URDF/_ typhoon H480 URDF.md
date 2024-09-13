TYPHOON H480 URDF
===

### 수정 ing

- link절 pose 다 0으로 주고 joint pose로 맞춤

```
<?xml version="1.0" ?>
<robot name="typhoon_h480">
  <!-- Typhoon H body -->
  <origin rpy="0 0 0" xyz="1.0 1.0 3.0"/>
  <link name="base_link">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <inertial>
      <mass value="2.02"/>
      <inertia ixx="0.011" ixy="0" ixz="0" iyy="0.015" iyz="0" izz="0.021"/>
    </inertial>
    <collision name="base_link_collision">
      <geometry>
        <box size="0.67 0.67 0.15"/>
      </geometry>
    </collision>
    <visual name="base_link_visual">
      <geometry>
        <!--mesh filename 경로 자세하게 적으니 URDF parsed OK-->
        <mesh filename="package://dam-drone/urdf/TyphoonH480/meshes/main_body_remeshed_v3.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="gazebo/red">
        <color rgba="0.8 0.0 0.0 1.0"/>
      </material>
    </visual>
    <gravity>true</gravity>
    <self_collide>false</self_collide>
  </link>

  <link name="cgo3_mount_link">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <inertial>
      <!-- place holder -->
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual name="cgo3_mount_visual">
      <geometry>
        <mesh filename="package://dam-drone/urdf/TyphoonH480/meshes/cgo3_mount_remeshed_v1.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="gazebo/red">
        <color rgba="0.8 0.0 0.0 1.0"/>
      </material>
    </visual>
  </link>
  <joint name="cgo3_mount_joint" type="revolute">
    <parent link="base_link"/>
    <child link="cgo3_mount_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="100" lower="0" upper="0" velocity="-1"/>
    <dynamics damping="1"/>
  </joint>

  <link name="cgo3_vertical_arm_link">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <inertial>
      <!-- place holder -->
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual name="cgo3_vertical_arm_visual">
      <geometry>
        <mesh filename="package://dam-drone/urdf/TyphoonH480/meshes/cgo3_vertical_arm_remeshed_v1.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="gazebo/red">
        <color rgba="0.8 0.0 0.0 1.0"/>
      </material>
    </visual>
  </link>
  <joint name="cgo3_vertical_arm_joint" type="revolute">
    <parent link="cgo3_mount_link"/>
    <child link="cgo3_vertical_arm_link"/>
    <origin rpy="-0.026 0 -0.10" xyz="0 0 0"/>
    <!--
    <controlIndex>6</controlIndex>
    -->
    <axis xyz="0 0 1"/>
    <limit effort="100" lower="-1e16" upper="1e16" velocity="-1"/>
    <dynamics damping="0.1"/>
  </joint>

  <link name="cgo3_horizontal_arm_link">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <inertial>
      <!-- place holder -->
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual name="cgo3_horizontal_arm_visual">
      <geometry>
        <mesh filename="package://dam-drone/urdf/TyphoonH480/meshes/cgo3_horizontal_arm_remeshed_v1.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="gazebo/red">
        <color rgba="0.8 0.0 0.0 1.0"/>
      </material>
    </visual>
  </link>
  <joint name="cgo3_horizontal_arm_joint" type="revolute">
    <parent link="cgo3_vertical_arm_link"/>
    <child link="cgo3_horizontal_arm_link"/>
    <origin rpy="0.026 0 -0.162" xyz="0 0 0"/>
    <!--
    <controlIndex>7</controlIndex>
    -->
    <axis xyz="-1 0 0"/>
    <limit effort="100" lower="-0.785398" upper="0.785398" velocity="-1"/>
    <dynamics damping="0.1"/>
  </joint>

  <link name="cgo3_camera_link">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <inertial>
      <!-- place holder -->
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual name="cgo3_camera_visual">
      <geometry>
        <mesh filename="package://dam-drone/urdf/TyphoonH480/meshes/cgo3_camera_remeshed_v1.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="gazebo/red">
        <color rgba="0.8 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision name="cgo3_camera_collision">
      <geometry>
        <sphere radius="0.035"/>
      </geometry>
    </collision>
    <sensor name="camera_imu" type="imu">
      <always_on>true</always_on>
    </sensor>
    <sensor name="camera" type="camera">
      <camera>
        <image far="15000" format="R8G8B8" height="360" hfov="2.0" near="0.05" width="640"/>
      </camera>
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <visualize>false</visualize>
    </sensor>
  </link>

  <joint name="cgo3_camera_joint" type="revolute">
    <parent link="cgo3_horizontal_arm_link"/>
    <child link="cgo3_camera_link"/>
    <origin rpy="-0.041 0.03 -0.162" xyz="0 0 0"/>
    <axis xyz="0 -1 0"/>
    <limit effort="100" lower="-0.0872665" upper="1.5708" velocity="-1"/>
    <dynamics damping="0.1"/>
  </joint>

  <link name="left_leg">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <inertial>
      <!-- place holder -->
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <collision name="collision">
      <geometry>
        <cylinder length="0.3" radius="0.012209"/>
      </geometry>
    </collision>
    <collision name="collision_bar">
      <geometry>
        <cylinder length="0.176893" radius="0.00914984"/>
      </geometry>
    </collision>
    <visual name="base_link_left_leg">
      <geometry>
        <mesh filename="package://dam-drone/urdf/TyphoonH480/meshes/leg2_remeshed_v3.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="gazebo/darkGrey">
        <color rgba="0.3 0.3 0.3 1.0"/>
        <!--원래 lighting ambient diffuse specular memissive 있는데 urdf에는 없어서 없앰 확인-->
      </material>
    </visual>
  </link>
  <joint name="left_leg_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_leg"/>
    <origin rpy="0.00026 -0.040515 -0.048" xyz="0 0 0"/>
    <axis xyz="-1 0 0"/>
    <limit effort="100" lower="0" upper="1" velocity="-1"/>
    <dynamics damping="1"/>
  </joint>

  <link name="right_leg">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <inertial>
      <!-- place holder -->
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <collision name="collision">
      <geometry>
        <cylinder length="0.3" radius="0.012209"/>
      </geometry>
    </collision>
    <collision name="collision_bar">
      <geometry>
        <cylinder length="0.176893" radius="0.00914984"/>
      </geometry>
    </collision>
    <visual name="base_link_right_leg">
      <geometry>
        <mesh filename="package://dam-drone/urdf/TyphoonH480/meshes/leg1_remeshed_v3.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="gazebo/darkGrey">
        <color rgba="0.3 0.3 0.3 1.0"/>
      </material>
    </visual>
  </link>
  <joint name="right_leg_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_leg"/>
    <origin rpy="0.00026 0.040515 -0.048" xyz="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit effort="100" lower="0" upper="1" velocity="-1"/>
    <dynamics damping="0.1"/>
  </joint>
  
  <link name="typhoon_h480/imu_link">
    <origin xyz="0 0 0" rpy="0 0 3.1415927"/>
    <inertial>
      <mass value="0.015"/>
      <inertia ixx="1e-05" ixy="0" ixz="0" iyy="1e-05" iyz="0" izz="1e-05"/>
    </inertial>
  </link>
  <joint name="typhoon_h480/imu_joint" type="revolute">
    <parent link="base_link"/>
    <child link="typhoon_h480/imu_link"/>
    <axis xyz="1 0 0"/>
    <limit effort="0" lower="0" upper="0" velocity="0"/>
  </joint>

  <link name="rotor_3">
    <origin xyz="0.211396 0.119762 0.082219" rpy="0 0 0"/>
    <inertial>
      <mass value="0.005"/>
      <inertia ixx="9.75e-07" ixy="0" ixz="0" iyy="0.000273104" iyz="0" izz="0.000274004"/>
    </inertial>
    <collision name="rotor_3_collision">
      <geometry>
        <cylinder length="0.128" radius="0.005"/>
      </geometry>
    </collision>
    <visual name="rotor_3_visual">
      <geometry>
        <mesh filename="package://dam-drone/urdf/TyphoonH480/meshes/prop_ccw_assembly_remeshed_v3.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="gazebo/blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <gravity>true</gravity>
    <self_collide>false</self_collide>
  </link>
  <joint name="rotor_3_joint" type="revolute">
    <parent link="base_link"/>
    <child link="rotor_3"/>
    <axis xyz="0.0446 -0.0825 1.8977"/>
    <limit effort="10" lower="-1e+16" upper="1e+16" velocity="-1"/>
    <dynamics damping="0.005"/>
  </joint>

  <link name="rotor_0">
    <origin xyz="-0.209396 0.122762 0.082219" rpy="0 0 2.09439510239"/>
    <inertial>
      <mass value="0.005"/>
      <inertia ixx="9.75e-07" ixy="0" ixz="0" iyy="0.000273104" iyz="0" izz="0.000274004"/>
    </inertial>
    <collision name="rotor_0_collision">
      <geometry>
        <cylinder length="0.128" radius="0.005"/>
      </geometry>
    </collision>
    <visual name="rotor_0_visual">
      <geometry>
        <mesh filename="package://dam-drone/urdf/TyphoonH480/meshes/prop_ccw_assembly_remeshed_v3.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="gazebo/blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <gravity>true</gravity>
    <self_collide>false</self_collide>
  </link>
  <joint name="rotor_0_joint" type="revolute">
    <parent link="base_link"/>
    <child link="rotor_0"/>
    <axis xyz="0.046 0.0827 1.8977"/>
    <limit effort="10" lower="-1e+16" upper="1e+16" velocity="-1"/>
    <dynamics damping="0.005"/>
  </joint>

  <link name="rotor_4">
    <origin xyz="-0.00187896 0.242705 0.0822169" rpy="0 0 0"/>
    <inertial>
      <mass value="0.005"/>
      <inertia ixx="9.75e-07" ixy="0" ixz="0" iyy="0.000273104" iyz="0" izz="0.000274004"/>
    </inertial>
    <collision name="rotor_4_collision">
      <geometry>
        <cylinder length="0.128" radius="0.005"/>
      </geometry>
    </collision>
    <visual name="rotor_4_visual">
      <geometry>
        <mesh filename="package://dam-drone/urdf/TyphoonH480/meshes/prop_cw_assembly_remeshed_v3.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="gazebo/blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <gravity>true</gravity>
    <self_collide>false</self_collide>
  </link>
  <joint name="rotor_4_joint" type="revolute">
    <parent link="base_link"/>
    <child link="rotor_4"/>
    <axis xyz="-0.09563 -0.0003 1.8976"/>
    <limit effort="10" lower="-1e+16" upper="1e+16" velocity="-1"/>
    <dynamics damping="0.005"/>
  </joint>

  <link name="rotor_1">
    <origin xyz="0.211396 -0.119762 0.082219" rpy="0 0 -2.09439510239"/>
    <inertial>
      <mass value="0.005"/>
      <inertia ixx="9.75e-07" ixy="0" ixz="0" iyy="0.000273104" iyz="0" izz="0.000274004"/>
    </inertial>
    <collision name="rotor_1_collision">
      <geometry>
        <cylinder length="0.128" radius="0.005"/>
      </geometry>
    </collision>
    <visual name="rotor_1_visual">
      <geometry>
        <mesh filename="package://dam-drone/urdf/TyphoonH480/meshes/prop_cw_assembly_remeshed_v3.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="gazebo/blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <gravity>true</gravity>
    <self_collide>false</self_collide>
  </link>
  <joint name="rotor_1_joint" type="revolute">
    <parent link="base_link"/>
    <child link="rotor_1"/>
    <axis xyz="0.0486 0.0811 1.8976"/>
    <limit effort="10" lower="-1e+16" upper="1e+16" velocity="-1"/>
    <dynamics damping="0.005"/>
  </joint>

  <link name="rotor_5">
    <origin xyz="-0.00187896 -0.242705 0.0822169" rpy="0 0 -2.09439510239"/>
    <inertial>
      <mass value="0.005"/>
      <inertia ixx="9.75e-07" ixy="0" ixz="0" iyy="0.000273104" iyz="0" izz="0.000274004"/>
    </inertial>
    <collision name="rotor_5_collision">
      <geometry>
        <cylinder length="0.128" radius="0.005"/>
      </geometry>
    </collision>
    <visual name="rotor_5_visual">
      <geometry>
        <mesh filename="package://dam-drone/urdf/TyphoonH480/meshes/prop_ccw_assembly_remeshed_v3.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="gazebo/blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <gravity>true</gravity>
    <self_collide>false</self_collide>
  </link>
  <joint name="rotor_5_joint" type="revolute">
    <parent link="base_link"/>
    <child link="rotor_5"/>
    <axis xyz="-0.033996 -0.0006 0.68216"/>
    <limit effort="10" lower="-1e+16" upper="1e+16" velocity="-1"/>
    <dynamics damping="0.005"/>
  </joint>

  <link name="rotor_2">
    <origin xyz="-0.209396 -0.122762 0.082219" rpy="0 0 2.09439510239"/>
    <inertial>
      <mass value="0.005"/>
      <inertia ixx="9.75e-07" ixy="0" ixz="0" iyy="0.000273104" iyz="0" izz="0.000274004"/>
    </inertial>
    <collision name="rotor_2_collision">
      <geometry>
        <cylinder length="0.128" radius="0.005"/>
      </geometry>
    </collision>
    <visual name="rotor_2_visual">
      <geometry>
        <mesh filename="package://dam-drone/urdf/TyphoonH480/meshes/prop_cw_assembly_remeshed_v3.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="gazebo/blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <gravity>true</gravity>
    <self_collide>false</self_collide>
  </link>
  <joint name="rotor_2_joint" type="revolute">
    <parent link="base_link"/>
    <child link="rotor_2"/>
    <axis xyz="0.0404 -0.0876 1.8976"/>
    <limit effort="10" lower="-1e+16" upper="1e+16" velocity="-1"/>
    <dynamics damping="0.005"/>
  </joint>
</robot>
```
