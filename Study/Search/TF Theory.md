TF (Transfrom)
===
### Example
> Reference : https://automaticaddison.com/coordinate-frames-and-transforms-for-ros-based-mobile-robots/

![image](https://user-images.githubusercontent.com/108650199/182310038-1f9873b2-a5de-42e8-bdd9-1018f2713c9e.png)

### 1. Coordinate Frame - .yaml 파일에 정의

#### 💥️ Relationship between Frames : Earth > map > odom > base_link 💥️

#### 1. base_link 
  - base_link라는 좌표 프레임은 모바일 로봇 베이스에 단단히 부착되어 있음
  - base_link는 임의의 위치 또는 방향으로 베이스에 부착될 수 있음
  
#### 2. odom : Local Pose
  - odom이라는 좌표 프레임은 세계 고정 프레임
  
##### &nbsp; <단점>
  - odom 프레임에서 모바일 플랫폼의 pose는 제한 없이 시간이 지남에 따라 drift 할 수 있음
  - 하지만 drift는 장기적인 관점에서 쓸모가 없음 왜냐? drift는 시간이 지날수록 더 커지니까
  
##### &nbsp; <장점>
  - 그러나odom 프레임에서 로봇의 pose는 연속적으로 보장
  - 즉, odom 프레임에서 모바일 플랫폼의 pose는 이산적인 점프 없이(특별히 튀는 현상이 없음) 항상 부드러운 방식
  
  - 일반적인 설정에서 odometry 프레임은 휠 주행 거리, 시각적 주행 거리 또는 관성 측정 단위와 같은 주행 거리 소스를 기반으로 계산
    - ex ) encoder, imu 등

##### &nbsp; <결론>
  - odom 프레임은 정확한 단기 로컬 참조로 유용하지만 드리프트로 인해 장기 참조에는 좋지 않은 프레임

#### 3. map : Global Pose
  - map이라는 좌표 프레임은 Z축이 위쪽을 가리키는 고정 프레임
  - 지도 프레임을 기준으로 한 모바일 플랫폼의 pose는 시간이 지남에 따라 크게 달라지지 않아야 함
  
##### &nbsp; <단점>
  - 맵 프레임은 연속적이지 않으므로 즉, 맵 프레임에서 모바일 플랫폼의 포즈는 언제든지 개별 점프로 변경될 수 있음

##### &nbsp; <장점>
  - 일반적인 설정에서 로컬라이제이션 구성 요소는 센서 관찰을 기반으로 맵 프레임에서 로봇 포즈를 지속적으로 다시 계산하므로 드리프트를 제거하지만 새로운 센서 정보가 도착하면 이산 점프를 유발
  
##### &nbsp; <결론>
  - 맵 프레임은 장기적인 글로벌 참조로 유용하지만 위치 추정기의 개별 점프로 인해 로컬 감지 및 작동에 좋지 않은 참조 프레임

### 2. Organize Frame 

> Reference : https://soohwan-justin.tistory.com/50

#### 1. map -> odom은 /tf를 통해 다뤄짐
```
Fusing Sensors for the /map frame (map -> odom)
```
- map -> odom으로의 좌표 변환을 위해서는 ekf/ukf_localization_node에서 "world_frame" 파라미터를 우리가 사용할 map frame의 이름으로 설정해야 함

<EX>
  
- Global pose estimates(AMCL)
  
- Absolute global pose data : GPS, Beacons, Global Visual Odometry

  
#### 2. odom -> base_link 좌표변환은 "odom"을 통해 보내지는 odometry message를 통해 다뤄짐
  
```
Fusing Sensors for the /odom frame (odom -> base_link)
```
  
- odom -> base_link로의 좌표 변환을 위해서는 ekf/ukf_localization_node에서 "world frame"파라미터를 우리가 사용할 odom frame의 이름으로 설정
  
<EX>
  
- IMU
  
- Visual Odometry
  
- Wheel Encoders
  
- laser-scan-matcher

#### 3. Example
- base_link_frame: 이 프레임은 로봇 그 자체이며, 어떤 센서든지 이 프레임이 기준이 됩니다. 이는 보통 로봇의 회전 중심에 위치합니다.
- odom_frame: 오도메트리를 report하기 위한 프레임 
-  map_frame: 이 프레임은 예를 들면 AMCL같이 로봇이 어디있는지 아는 시스템으로부터의 global position을 report하기 위해 사용되며, 만약 외부의 위치추정 시스템을 사용하지 않으면 이 옵션은 무시
- world_frame: world에 있는 로봇의 절대 좌표를 얻기 위해 이전의 프레임들 중 어떤 프레임을 참조할지를 정함
  
##### &nbsp; 만약 외부의 위치 추정 시스템이 없을 때의 Frame 
  ```
  base_link_frame: base_link
  odom_frame: odom
  world_frame: odom
  ```
