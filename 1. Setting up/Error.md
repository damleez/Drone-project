ERROR
===

### 1. package 못 찾을 때

```
source ~/catkin_ws/devel/setup.bash
rospack profile
rospack find dam-drone
```

### 2. robot state,, joint,, 어쩌고들

```
sudo apt-get install ros-noetic-controller-manager joint-state-publisher
sudo apt-get install ros-noetic-robot-state-publisher ros-noetic-robot-state-controller
```

### 3. gazebo sensor 불러올때는 lib없는 경우도 있음
- velodyne 패키지랑 velodyne simulator 패키지는 다른 경우라서 따로 받아줌
```
git clone https://github.com/lmark1/velodyne_simulator.git
```

- velodyne simulator에는 libhector~가 없어서 gps랑 imu 못써서 rostopic list에도 안나옴
```
sudo apt-get install ros-noetic-hector-gazebo
```
