Trouble Shooting
===

## <rotor_simulation>
### 1. Octomap Error
> 오류
```
Could not find a package configuration file provided by "octomap_msgs" with~
```

> 해결
```
sudo apt install ros-melodic-octomap-ros
```

### 2. Glog Error
> 오류
```
Failed to find glog - Could not find glog include directory, set
  GLOG_INCLUDE_DIR to directory containing glog/logging.h
```

> 해결
```
sudo apt install libgoogle-glog-dev 
```
- 또한,
```
sudo apt-get install libgflags-dev
sudo apt install libgoogle-glog-dev
sudo apt-get install protobuf-compiler libprotobuf-dev
```

## <gazebo_launch>
### 1. Dummy Link Error
> 오류
```
The root link base_link has an inertia specified in the URDF, but KDL does not support a root link with an inertia. As a workaround, you can add an extra dummy link to your URDF.
```
> 해결

- base_link 이외의 기초가 되는 link 및 joint 생성

```
<link name="base_link_zero"/>

<joint name="base_link_zero_joint" type="fixed">
  <parent link="base_link_zero"/>
  <child link="base_link"/>
  <origin rpy="0 0 0" xyz="0 0 0"/>
 </joint>
 ```
### 2. Propeller Fix Error
> 오류

![default_gzclient_camera(1)-2022-08-17T15_24_10 138852](https://user-images.githubusercontent.com/108650199/185049326-c2ca9032-c9ca-437f-8d67-c7310f38abe7.jpg)

- 날개가 붙어있지 않음
  - TF의 문제인듯 함

> 해결
- 1. Scale을 그대로 복원
- 2. Link의 Inertial, Visual, Collision은 각각으로 생각해야하며 한 덩어리가 아님
  - 즉, Link의 origin설정해줘도 inertial, visual, collision의 origin설정을 안함으로써 생기는 문제 발생
  - 날개는 0을 기준으로 얼만큼 떨어진 mesh파일 visual을 가지고 있으므로, visual origin설정을 0으로 땡겨옴으로써 해결해야함
