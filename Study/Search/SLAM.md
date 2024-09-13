SLAM(Simultaneous localization and mapping)
===

### 1. 정의
- SLAM은 동시적 위치추정 및 지도작성이라고도 불리고, 자율주행 차량에 사용되어 주변 환경 지도를 작성하는 동시에 차량의 위치를 작성된 지도 안에서 추정하는 방법
- SLAM을 통해 미지의 환경에 대한 지도를 작성하며, 엔지니어는 지도 정보를 사용하여 경로 계획(Path planning) 및 장애물 회피 등의 작업 수행

### 2. 작동 방식

![image](https://user-images.githubusercontent.com/108650199/191417069-2bf07584-1f3e-4846-b9b0-4b9f6c5eea6e.png)

- 프론트 엔드 : 센서 신호 처리
- 백 엔드 : 자세 그래프 최적화로 센서와는 무관

### 3. SLAM의 종류
#### 3-1. visual SLAM
##### - 정의
- visual SLAM은 vSLAM이라고도 하며, 카메라 및 기타 영상 센서로 획득한 영상을 사용
- 일반 카메라(광각, 어안 및 구형), 겹눈 카메라(스테레오 및 멀티), RGB-D 카메라(depth 및 TOF)를 사용

![image](https://user-images.githubusercontent.com/108650199/191417951-4dc6e3c4-ab6e-46b2-ae62-1ffbdeb9a32c.png)

- 장점 : 저비용, 카메라에서 제공되는 정보 방대하므로 이를 토대로 랜드마크(이전에 측정한 위치) 감지
- 단점 : 카메라를 이용하여 SLAM을 수행할 경우 시간이 지남에 따라서 오차가 누적되므로 최종 생성된 지도와 위치 추정의 오차가 매우 커지는 경우 발생
  - 오차가 발생하는 원인은 주변 환경과 조도 변화에 따라서 센서 관측치(observation)에 대한 노이즈 또는 모호성(ambiguity)이 존재하기 때문
  - 이러한 문제점을 극복하기 위한 Visual SLAM 기술은 크게 기하하적 방법, 학습을 이용한 방법(딥러닝) 그리고 학습과 기하학적 방법을 융합한 하이브리드 방법(기하학+딥러닝)으로 나뉨

##### - 기하학적 visual SLAM 방법론
임의의 환경에서 획득한 영상 시퀀스로부터 기하학적 계산을 통해 카메라의 위치 및 3차원 지도를 생성하면서 발생하는 오차를 줄이기 위한 방법은 필터링(filtering)과 최적화(optimization) 기법으로 나눔
1) 필터링(filtering) : 칼만 필터(Kalman filter)와 입자 필터(particle filter)기반의 방법으로 구분
2) 최적화(optimization) 기법 : Feature-Based(특장점 기반) SLAM과 Direct SLAM으로 구분
- Feature-based VSLAM : 영상으로부터 특징점을 추출하고 이를 영상 시퀀스에서 추적하여 초기 카메라의 위치를 알고 계산하고 3차원 지도를 생성
  - 그리고 3차원 지도를 구성하는 랜드마크들의 위치들을 카메라의 추정된 자세로 재 투영(re-projection)시켜서 영상으로부터 추적된 특징점의 좌표와의 거리를 최소화 
  - 징점의 오정합(false matching)으로 발생하는 문제들을 RANSAC(Random Sample Consensus) 기반의 방법을 활용하여 제거하는 것이 가능
    - 예시 : SIFT(Scale-invariant Feature Transform), SURF(Speed Up Robust Features), Oriented FAST and Rotated BRIEF (ORB)
- Direct SLAM : 두 장의 영상으로부터 카메라의 움직임과 환경에 대한 3차원 정보를 획득하기 위해서 첫 번째 영상을 두 번째 위치에서의 영상으로 변환하였을 때, 실제 획득한 두 번째 영상과의 밝기 차이를 최소화하도록 최적화를 수행하여 개선하는 Direct SLAM 방법이 있음 
  - 또한, Direct SLAM은 특징을 추출하지 않고 전체 이미지를 사용
  - Direct SLAM 방법의 경우 처리속도가 느리나 환경을 조밀하게 모델링하는 것이 가능하며 특징점이 없는 균질한(homogeneous) 환경에서 성능이 우수
    - 예시 : DTAM, LSD-SLAM, SVO or DSO 

![image](https://user-images.githubusercontent.com/108650199/191418643-d4a55bc4-9105-466b-9cbe-78ecd2c92191.png)

##### - Feature-based SLAM vs Direct SLAM

- 일반적인 VSLAM 시스템의 순서도

![image](https://user-images.githubusercontent.com/108650199/193718398-109f1373-0b27-480b-b806-056d081060d3.png)

#### 3-2. lidar SLAM
##### - 정의
- LiDAR(Light Detection and Ranging)은 레이저 센서를 주로 사용하는 방법
- 레이저는 카메라, ToF센서보다 훨씬 더 정밀하며 자율주행 차량과 드론처럼 빠르게 이동하는 물체와 관련된 응용 사례에 사용
- 레이저 센서에서 얻어지는 출력값은 일반적으로 2차원 or 3차원(포인트 클라우드) 데이터
- 레이저 센서 포인트 클라우드를 사용하면 고정밀 거리 측정이 가능하며, SLAM을 적용한 지도 생성에도 매우 효과적

##### - [단점](https://ignitarium.com/visual-slam-possibilities-challenges-and-the-future/)
1) 처리하는 데이터 유형에 따라 고성능의 컴퓨팅 성능 필요
2) LiDAR와 관련한 하드웨어는 비쌈
3) perception(like object detection ...) 복잡
4) sementic 정보가 부족

##### - Tightly / Loosley couped system

#### SLAM Localization

![image](https://user-images.githubusercontent.com/108650199/191474191-60ad0d65-4392-45a5-a62d-065a6175c529.png)

#### SLAM Perception

#### SLAM Planning

#### SLAM Mapping
