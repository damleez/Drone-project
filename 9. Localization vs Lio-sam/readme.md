## Launch

![Peek 2022-09-14 15-32](https://user-images.githubusercontent.com/108650199/190077466-fe2c6849-02c2-49a5-9eb2-099e2a2cd87e.gif)

- GAZEBO
```
roslaunch dam-drone xacro.launch enable_lidar:=true use_mavros:=true
```
- LIO-SAM
```
roslaunch lio_sam run.launch
```
- CONTROL
```
rosrun dam-drone dam_set_xxx_node
```
- EKF
  - odometry/odometry
```
roslaunch robot_localization ekf_template.launch
```
- 🌟️ EKF 이후 NAVSAT 🌟️
  - odometry/navsat
```
roslaunch robot_localization navsat_transform_template.launch
```

## Localization

![image](https://user-images.githubusercontent.com/108650199/190043264-1b4736d2-c0ec-4464-9f1c-b12f8b6a0751.png)

![image](https://user-images.githubusercontent.com/108650199/190040374-3944a450-d0ae-4358-9146-ad8286ecbac2.png)
