# Drone-project

### Keyboard Setting

![image](https://user-images.githubusercontent.com/108650199/180127742-bdec069b-85eb-42c7-8930-e402028cab51.png)

---

### Setting for Beginner
- First step
- [x] sudo apt-get update
- [x] sudo apt-get upgrae
- [x] sudo apt-get ssh net-tools vim terminator (나중에 vpn도 깔아야함)
- [x] In nuc, change language to ENGLISH (언어팩 > English up)
- [x] install ROS (In nuc, snaped build 오류시 sudo apt-get purge snaped)
- [x] mavros install 
- [x] velodyne 3D install
- [x] microstrain mips install (IMU) (git clone 후 catkin build)
- [x] LioSAM install
- [x] IMU port symbolic link (> sudo udevadm trigger 로 확인)
- [ ] Pixhawk port symbolic link (밖에서)

- Check sensor data
- [x] roslaunch velodyne point cloud
- [x] roslaunch microstrain_mips ~ :: port 

* package에 필요한 의존성 오류시 한번에 다운 > rosdep install

---

### To Do List
~ 7/11
- [x] Xavier Setting
    - SDK Manager를 통해 Xavier 설치
    - ROS Noetic 설치 on Xavier
    - Mavros 설치 on Xavier
    - Pixhawk와 Xavier연결
    - roslaunch 자동실행
    - USB devide 고정 (dev ID 입력, symbolic link 생성, 권한부여)
    - Opencv install & build
    - Lidar & IMU 연결 및 설치 (ref : velodyne ros wiki)
    
~7.21    
- [X] ROS custom package create(input image.cpp, manipulation.cpp)
    - basic function<br>
        - Spacebar : arming, set mode to 'GUIDED' <br>
        - T : takeoff // height select <br>
        - E : set local postition (move) => guided mode <br>
        - C : rotate <br>
        - R : take off 위치로 return(rtl)<br>
    - additional function<br>
        - Q : brake <br>
        - W,A,S,D : move control<br>
        - F : exit<br>
        - G : get current position<br>

~ 7.29
- [X] Simulation env Setting URDF(+TF, +Plugin)
    - Velodyne Lidar 
    - Depth Camera
    - Mono Camera
    - IMU
    - Gazebo drone model (typhoon)
    - World map   
    
![frames](https://user-images.githubusercontent.com/108650199/182068401-c548b6b0-7e18-48de-bb0c-089ec3833cb2.png)

~ 8.22

![image](https://user-images.githubusercontent.com/108650199/185823484-dd54d991-06ad-4265-a0e4-db0abba4bfef.png)

- [X] Specific Sensor Plugin
    - lanch file argument 추가
    - sunbae-urdf 의 drone.launch 및 iris_base.xacro와 component_snippets.xacro 참고
- [ ] ~~Uploading the Drone Model on the Ardupilot~~
    - parameter 수정
    - hexa, odo copter... 등으로 변경
- [X] Ardupilot on the IRIS
- [X] Turn Lidar
    - yaml 파일 생성
    
~ 8.30
- [X] LiDAR Position Change
    - transmission position 추가
- [X] Sensor Study for SLAM
    - Velodyne LiDAR
    - Oster LiDAR
    - IMU (6-axis, 9-axis)
    - IMU : AHRS, INU
    - GPS : GNS
    - Filter : KF, EKF

~ 9.8
- [X] Robot localization code analysis
    - [robot localization git](https://github.com/cra-ros-pkg/robot_localization)
        - launch
        - cpp
        - hpp

~ 9.19
- [X] Sensor pose comparison
    - In simulation, lidar/imu/gps
        - lidar+imu > Lio-sam pose estimation
        - imu+gps > robot localization (in gazebo with ros)
- [X] Lio-sam and robot localization pose 비교 > 두 가지 방법 (pose = position+orientation)
    - rviz 에서 비교 가능
    - topic 에서 비교 가능 

~ 9.26
- [X] IMU error down
    - odometry/gps vs odometry/navsat(>원인 imu not gps)
    - imu config setting
        - parameter
        - config

~ 10.3
- [X] GPS Lio-sam vs Non GPS Lio-SAM vs Robot localization
    - GPS on Lio-sam
    - Edit param.yaml and launch file

~ 10.12
- [X] SLAM Review
    - SLAM Paper Review
        - Type of SLAM (Lidar or Visual)
        - Optimization
        
~ 10.21
- [X] SLAM keywords study
    - Green book chapter 3
        - transform/translation/quaternion/Euler
        - Eigen library
    - Non-linear optimization
        - Gauss-Newton
        - levenberg marquardt ...
    - Point cloud registration
        - ICP, GICP, NDT ...
        - In LOAM paper, feature extration
    - Pose graph vs graph optimization
    - factor graph
    - Back-end method
        - isam
        - g2o

~ 10.28
- [X] SLAM Keywords study & HDL graph slam
    - Green book chapter 4
        - Lie Group 
        - Lie Algebras
    - Green book chapter 10 
        - Filter based back-end
        - sparse matrix application
    - IN LOAM paper, Lidar Odometry
        - optimization (levenberg marquardt)
    - HDL Graph SLAM
        - Install and play bag file
        - My SLAM application

~ 11.11
- [X] HDL graph SLAM code analysis
    - [HDL graph SLAM git](https://github.com/koide3/hdl_graph_slam)
        - Concept mapping
        - In apps file, nodlet code

~ 11.16
- [X] Pointcloud Registration Method
    - Pointcloud registration
        - ICP
        - GICP
        - NDT
        
~ 11.28
- [X] KD-tree, Octree
- [X] LOAM-line Paper Review
    - LOAM
    - LeGO-LOAM
    - LIO-SAM
    
~ 12.7
- [X] LOAM-line Paper Formula Study
- [X] LIO-SAM Code analysis
    - In src file, featureExtraction.cpp
    - imageProjection.cpp
    - imuPreintegration.cpp
    - mapOptimmization.cpp

~ 12.14
- [X] LIO-SAM Cod analysis
    - time을 중점으로 analysis
    - overall

~ 12.23
- [X] Edit LIO-SAM bag.file
    - ~~맵 뒤틀림~~
    - Noise & LiDAR pose nose
    - parameter edit

---

### 2023

~ 1.10
- [X] Edit LIO-SAM bag.file
    - extrinsicRot, RPY
    - 맵 뒤틀림
        - roll > deskewing
        - timesync
    - Noise
        - updateInitialGuess()

~ 1.13
- [X] Edit LIO-SAM bag.file
    - UpdateInitialGuess()
- [X] tf, keyframe with sunbae
    - encoder TF
    - map에서 keyframe 추출
- [X] HDL Global Localization 
    - library, parameter, algorithm ..(like teaser, NDT)

## 🎉 발표 끝 ! 🎉
