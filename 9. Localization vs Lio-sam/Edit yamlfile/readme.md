### 0. origin

![19 origin](https://user-images.githubusercontent.com/108650199/190999426-0e4d88e5-0c37-428b-bed2-7c803b8fb21b.png)

### 1. like lio param
- lio sam의 imu config,queue / odom queue / process noise 맞춤

![19 like liosam param](https://user-images.githubusercontent.com/108650199/190998961-34cbb451-8b80-476c-a5b6-b4e08a9a57f4.png)

### 2. Frequency
5해도 50해도 똑같음

### 3. Covariance 
#### 1) x,y,z 값인 1 1 10 으로 맞추기
- 오차 커짐

![19 covariance big](https://user-images.githubusercontent.com/108650199/191002389-3b2b72ba-2d87-457c-a23b-7171b59d50ee.png)

#### 2) x,y,z 빼고 0.5로 맞추기

![19 covariance (xyz빼고)0 5](https://user-images.githubusercontent.com/108650199/191003036-68827bab-72b9-42d4-a842-cc94b07a8abc.png)

#### 3) 0.01로 다 맞추기

![19 covariance 0 01 ](https://user-images.githubusercontent.com/108650199/191003638-07b790f4-289f-45c1-b59f-6afc1d3b534f.png)

- x,y,z는 큰 값이 맞는 듯 함
- 나머지를 하나씩 바꿔보기

### 4. Wolrd frame map > odom
- world map은 상관 없는듯

![19 world frame   odom](https://user-images.githubusercontent.com/108650199/191004108-36766355-4e3f-46dc-9e98-5df4cfebe272.png)

### 5. IMU differential > true
- 포즈 정보를 포함하는 위에 정의된 각 센서 메시지에 대해 사용자는 포즈 변수를 차등적으로 통합해야 하는지 여부를 지정할 수 있음
- 주어진 값이 true로 설정되면 해당 센서의 시간 t에서의 측정에 대해 먼저 시간 t−1에서의 측정값을 빼서 결과 값을 속도로 변환
- 이 설정은 로봇에 두 개의 절대 포즈 정보 소스가 있는 경우 특히 유용(예: 주행 거리 측정 및 IMU의 요 측정)
- 이 경우 입력 소스의 편차가 올바르게 구성되지 않으면 이러한 측정이 서로 동기화되지 않고 필터에서 진동을 유발할 수 있지만 둘 중 하나 또는 둘 다를 차등적으로 통합하면 이 시나리오를 피함
- 사용자는 방향 데이터에 이 매개변수를 사용할 때 주의
- 속도로 변환하면 방향 상태 변수에 대한 공분산이 제한 없이 증가하기 때문(절대 방향 데이터의 다른 소스가 융합되지 않는 한)
- 모든 포즈 변수가 0에서 시작하기를 원할 경우, _relative 매개변수를 사용하십시오.

- 

![20 imu differential](https://user-images.githubusercontent.com/108650199/191146445-3eed3d24-e71b-4735-8c65-7f0194278f8a.png)

### 6. Sensorbase.xacro GPS sensor update rate setting 1 > 500 (because imu update rate is 500)
- 역시 정확한 gps update rate 비율 높으니까 오차 많이 줄어져서 나옴

![20 updaterate setting](https://user-images.githubusercontent.com/108650199/191148789-b7a44fb5-2d7f-44a0-a589-bec8be403714.png)

### 7. imu update rate 400, process noise covariance origin(not lio-sam covariance)
- 원래의 covariance를 쓰니 훨씬 더 안정적

![20 origin covariance imu 400](https://user-images.githubusercontent.com/108650199/191151540-b93fecb8-981d-4532-b31c-77e15a6be599.png)

- repeat 해도 안정적

![20 origin covariance imu 400 repeat](https://user-images.githubusercontent.com/108650199/191151831-4af98d54-b2c0-412d-939a-ec613dc5dddc.png)
