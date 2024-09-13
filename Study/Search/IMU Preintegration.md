IMU PRE INTEGRATION
===

### 1. IMU의 [preintegration](https://limhyungtae.github.io/2022-04-01-IMU-Preintegration-(Easy)-1.-Introduction/) 하는 이유
- IMU data는 Hz가 매우 빠르기 때문에 많은 data를 취득할 수 있음 (than lidar, camera ...)
  - continuous-time sys인 우리 주변의 환경에서 움직임을 discrete time sys상에서 잘 묘사하기 위해서
  - but, 3차원에서 pose를 기술하기 위해서는 6개의 파라미터가 필요한데, IMU 측정으로부터 추정한 pose를 factor로 추가하면 파라미터 수가 너무 많아짐 
  - ex) imu data 5000개의 data취득을 했다면,
    - 각 IMU의 time step에 대한 pose를 묘사하기 위해서는 3D translation를 위한 파라미터 3개와 3D rotation을 위한 파라미터 3개로 총 6개가 필요하게 한데,
    - 그러면 총 5,000 × 6 = 30,000 여개의 parameter가 필요한 상황 
    - LiDAR, camera의 keyframe마다 pose 사이사이에 수백 개의 IMU factor가 생성되므로 이는 연산량이 많아져서 실시간성을 위협
    
    ![image](https://user-images.githubusercontent.com/108650199/191436638-47341994-a6d8-4aa7-b47d-018e86bc7b13.png)

#### - 전개 방식

![image](https://user-images.githubusercontent.com/108650199/191442444-9b794a26-1b66-4eb8-8847-6a27bc66195c.png)

### 2. [keyframe](https://limhyungtae.github.io/2022-04-01-IMU-Preintegration-(Easy)-2.-Preliminaries-(1)-Keyframe/)
- keyframe은 말그대로 “중요한 (key) frame”을 뜻함
- 여기서 “중요하다”는 말은 주어진 두 frame 사이에 relative pose를 추정할 때
- 주변 환경의 기하학적 특성을 잘 묘사할 수 있는 reliable & repeatable features가 풍부하여 충분히 정확한 pose 추정이 가능하다는 것을 의미
- keyframe을 선정하는 주된 이유는 사용하는 sesnor의 모든 data를 전부 사용하여 SLAM을 하려 하면 많은 메모리를 필요로 하기 때문이라고 생각

- ex) 3D LiDAR sensor는 대부분 10 Hz로 3D point cloud를 취득하고, camera sensor의 같은 경우에는 약 30 Hz로 image를 취득
- 그런데 LiDAR sensor를 활용해서 큰 도심 환경을 매핑하기 위해 약 1시간 가량 데이터를 취득했다고 가정하면, 1시간 동안 약 36,000개의 frame을 얻음
- 각 frame은 약 10만 여개의 points로 구성되어 있는데 (64 채널 기준. Velodyne HDL 64E의 경우 약 130,000 여개의 points를 매 frame마다 획득함),
- 이 raw range를 (x, y, z) format으로 파싱하면 각 point 당 3개의 float을 필요
- 그러면 어림잡아도 raw data를 저장하는 데에만 36,000 * 130,000 * 3 * 4하게 되면 약 56 GB 정도의 메모리가 필요
- (물론 이해를 돕기 위한 예시일 뿐, 정확하지 않음, 실제로 SLAM을 할 때는 voxelization 등을 활용해서 최소한 points만 저장하기 때문)

- 따라서 이러한 문제를 해결하기 위해서 대부분의 SLAM 알고리즘들에서는
- a) 이전 keyframe 기준 로봇이 어느 정도 이상 움직였거나 (하지만 그 거리가 “적당히” 멀어야 함, Keyframe 사이의 거리가 너무 멀어지게 되면 pose 추정을 하는 것이 오히려 부정확해지기 때문)
- b) 이전 keyframe 기준 어느 정도 시간이 흘렀을 경우 다음 keyframe을 생성

![image](https://user-images.githubusercontent.com/108650199/191440834-a0eff2cd-dcfa-4287-a6ef-484ebae1a60b.png)

- 결과적으로 keyframe은 위의 그림과 같이 sensor로 매번 취득되는 frame 중에서 어느 정도의 간격을 두고 선별
- 원 논문에서는 인접한 두 keyframe을 i와 j로 표기
- 따라서 preintegration의 문제 정의는 “두 keyframe 사이의 수십~수백여개의 IMU data (그림 상의 x 표시)를 어떻게 하나의 factor (그림 상의 파란 ■)로 표현할 수 있는가”로 정리
