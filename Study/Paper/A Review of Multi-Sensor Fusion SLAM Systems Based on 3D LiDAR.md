A Review of Multi-Sensor Fusion SLAM Systems Based on 3D LiDAR
===
## 0. Abstract
- LIDAR 기반 SLAM 시스템은 역동성이 높거나 features가 희박한 극한 환경에서 위치 파악 및 매핑 효과가 저하되고 영향을 미침
- 최근 몇 년 동안 보다 안정적이고 강력한 시스템을 얻기 위해 LIDAR 기반 다중 센서 융합 SLAM 작업이 많이 등장
- 본 논문에서는 SLAM의 기본 개념과 다중 센서 융합의 필요성을 요약한 후 융합 센서의 종류와 데이터 결합 방법을 기반으로 4가지 측면에서 다중 센서 융합의 기본 원리와 최근 연구 소개
  - 5가지 오픈 소스 알고리즘의 성능을 비교

## 1. 본론
### Introduction
- 정밀한 positioning은 자율적으로 작업을 완료하는 핵심 기술 중 하나 > 단일 센서보다는 다중 센서 
- 3D LiDAR를 이용한 야외 환경에서 주로 사용
  - 3D LiDAR는 풍부한 매칭 방법과 프레임 간의 매칭을 위한 point cloud 제공
  - 이미지나 odometry와 융합하여 위치 정확도 높임 (다중 센서)
- SLAM system은 정확한 위치 및 방향 추정이 필수적 (=orientation)
  - 단일 센서로는 부족하므로 문제를 해결하기 위해 IMU, GPS, MEMS 및 UWB와 같은 많은 보조 센서가 측위 시스템에 추가 (=다중 센서) 
- SLAM 관련 리뷰에는 Front-end Matching, Closed-loop Detection, Back-end Optimization, mapping과 같은 핵심 모듈을 기반
#### <목차>
- 1장 : localization system에서 다중 센서 융합의 필요성
- 2장 : SLAM이 해결해야 할 기본적인 문제와 고전적 프레임워크 제시
- 3장 : loosely coupled system의 관련 work를 센서 유형에 따라 2분류로 검토
- 4장 : tightly coupled system의 관련 work 검토

## 2. SLAM(Simultaneous Localization and Mapping) System

> Feature-based fusion SLAM framework

![image](https://user-images.githubusercontent.com/108650199/193735383-79c155a9-414f-4fed-a5b7-05f075ff2224.png)

- SLAM 문제는 localization과 mapping이라는 두 개의 독립적인 모듈에서 두 가지를 통합하는 완전한 시스템으로 발전

> The main components of a SLAM system

![image](https://user-images.githubusercontent.com/108650199/193735482-02223b5f-3d39-40bb-91d8-3d3529a39f15.png)

- Front-end : 현재 프레임 포즈를 실시간으로 추정하고 해당 맵 정보를 저장하는 역할
- back-end : 대규모 포즈와 장면 최적화를 담당
- closed loop detection : 로봇이 방문한 장면을 식별하고 글로벌 규모의 드리프트 수정을 트리거하는 데 도움이 되는 SLAM의 주요 문제 중 하나
- Large-scale global optimization : 대부분의 최신 다중 센서 융합 기술은 information complementation, local pose fusion, and multi-data source filtering을 통해 odometry 시스템의 고정밀 및 낮은 드리프트를 달성하기 위해 front-end에서 작동

> Classification of parts of the work and the relationship between them

![image](https://user-images.githubusercontent.com/108650199/193736613-49be9740-2469-4193-a164-6b54632d57ef.png)

- SLAM에서 가장 일반적으로 사용하는 센서 : LiDAR, camera, imu 
  - 이 3개의 센서 조합을 주로 사용하며 Loosley or tightly coupled system 두 가지로 나뉨 
  - Loosley coupled system : 계산량이 적고 시스템 구조가 간단하며 구현이 용이 but 위치 정확도 한계
  - Tightly coupled system : 계산 집약적이어서 구현이 어렵지만 복잡하고 변화무쌍한 환경에서 보다 정확한 상태 추정을 얻음

## 3. Multi-Sensor Loosely Coupled System Based on LIDAR
- Loosley coupled system은 주로 multi-sensor-based stage pose estimation, raw data-based information complementarity, and sensor-assisted pose constraints 3가지 용도로 사용

#### 3-1. LI(LiDAR-IMU) Loosley coupled system

![image](https://user-images.githubusercontent.com/108650199/193737700-6097cc71-808a-4d20-a75a-65d4c24ea594.png)

##### LOAM
- 대부분의 loosley coupled system은 초기 논문에서 나타남 > 가장 인기 있는 예시가 LOAM algorithm
  - LOAM은 복잡한 point cloud에서 유효한 edge 및 plane feature point를 추출한 것
  - 또한, point-to-line and plane 거리는 오류 함수를 구성하고 포즈의 nonlinear optimization problem을 해결하는 데 사용 
- LOAM은 6축 IMU의 자이로스코프와 가속도계의 통합 연산을 사용하여 이전 포즈를 획득하여 LIDAR odometry의 정확도를 더욱 향상
- 💥️ 그러나 LOAM에는 백엔드에서 루프 폐쇄 감지 및 전역 포즈 최적화가 없음 💥️ 
- 이 섹션에서 제시된 작업은 센서 데이터 융합에 중점을 둘 뿐만 아니라 프런트 엔드의 포인트 클라우드 등록 개선 및 백 엔드의 전반적인 최적화에 중점

##### LeGO-LOAM
- LeGO-LOAM : point cloud registration 속도를 높이기 위해 데이터 전처리에 point cloud clustering 및 ground segmentation을 도입
  - point cloud registration : 포인트 클라우드 정합
  - point cloud clustering : 포인트 클라우드 군집화
    - clustering : 머신러닝 기법 중 하나로 데이터셋들을 특정 기준으로 묶어 여러개의 군집(Cluster)로 만드는 방법을 의미
    - 여기서 기준은 거리, 색, feature, point cloud 등
    - 중심 기반, 계층 정의, 밀도 기반 군집화 등이 있음
  - ground segmentation : 지면 추출 방법
- 동시에 간단한 가속 공식을 사용하여 포인트 클라우드 왜곡 보정을 위한 IMU 데이터를 처리하고 priori 포즈를 제공

##### CSS-based LOAM 및 ALeGO-LOAM
- LOAM의 feature quality를 향상
- 곡률 스케일 공간 방식과 적응형 클라우드 샘플링 방식이 제시되어 더 정확한 에지 포인트와 평면 포인트를 추출
- 그러나 Loosley coupled system은 IMU 자체의 측정 바이어스의 영향을 효과적으로 배제하지 않으며 IMU는 보조 수단에 불과함
  - IMU의 단점을 효과적으로 없애지 못하며 부가적인 역할밖에 못해서 별로이다라고 하는 듯 

##### improved ICP and GPU acceleration
- ICP는 SLAM front-end에서 널리 사용 
  - 최신 연구[18]는 프론트 엔드 ICP 알고리즘에서 대칭 KL 발산을 소개
  - 최적화 대상에는 점 사이의 거리뿐만 아니라 분포 형태의 차이도 포함
- GPU 가속 : front-end 의 실시간 성능과 계산 정확도를 보장

##### Back end optimization
- 👉️ LOAM 자체가 front-end에 좀 더 집중되어있음 > 따라서, back-end 최적화하는 최신 논문도 나오게 됨
- 논문① 범주형 특징점을 기반으로 하는 MULLS ICP(Multi-metric Linear Least Squares Iterative Closest Point) 알고리즘을 사용하여 셀프 모션을 효율적으로 추정하고 서브맵 기반 PGO(포즈 그래프 최적화) 백엔드 최적화를 구성
- 효과적인 closed looped detection은 SLAM의 중요한 절차
- 논문② 포인트 클라우드의 기하학적 및 강도 정보가 인코딩되고 전역 포인트 클라우드 디스크립터가 회전 불변 루프 폐쇄 매칭 알고리즘을 구현하도록 설정되어 SLAM back-end에 대한 적절한 최적화 타이밍을 명확히 함
- 논문③ 포인트 클라우드의 모든 키 프레임으로 변환된 2D 히스토그램은 현재 프레임과 과거 키 프레임 간의 유사도를 계산하여 루프 폐쇄가 발생하는 위치를 결정

##### LOAM-Livox
- 기존의 VLP같은 mechanical lidar가 아니라 solid lidar를 이용한 SLAM
- 시스템은 규정되지 않은 포인트 클라우드를 제거하고 선 및 표면 피처를 추출
- 포즈는 선과 표면 거리의 잔차를 구성하여 반복적으로 해결
- But, IMU 사용 X

##### <LI loosley coupled system 정리>
- IMU의 loosley 결합은 포인트 클라우드의 왜곡 보정을 위해 IMU 데이터를 처리하고 priori pose를 제공
- 따라서, 센서 융합의 효과는 제한적
  - 따라서, LI loosley coupled system의 존재하는 알고리즘은 front-end와 back-end 개선이 대부분
- 최근 몇년간의 LI loosley coupled system은 system의 완성과 optimization에 초점
- 이는 정확한 front-end matching과 효율적인 back-end optimization을 찾는 것이 주요한 work
  - 이 부분의 작업은 데이터 융합에 크게 효과가 있는 것은 아니지만 후속 work을 위한 안정적인 플랫폼과 인터페이터 제공

#### 3-2. LVI(LiDAR-Visual-IMU) Loosley coupled system
- LiDAR odometry의 성능 저하는 구조화되지 않고 반복적인 환경에서 발생
  - IMU를 fusing해서 localization해도 좋아지진 않음
- 🌟️ 왜 LiDAR와 Visal을 섞었는가? 🌟️
  - Visual은 lcoalization을 위해 edge or plane같은 특정 구조적인 features가 필요 없음
  - but, visual은 깊이 정보를 직관적으로 알 수 없음
  - but, lidar는 point cloud에서 x,y,z 및 깊이를 알 수 있음 > 당연함 3D LiDAR의 점 군 집합이 point cloud임ㅊ
- 따라서 camera+lidar를 loosley하게 결합하면 상호보완적
  - LVI loosely coupled system의 LO 그리고 VO는 독립적이지만 pose correction 및 부드러운 추적을 위해 서로 위치 정보 공유

##### V-LOAM
- LOAM저자가 LOAM에 단안 카메라 tracking을 IMU와 결합하여 알고리즘 확장
- LIDAR 스캔 매칭을 위한 시각적 관성 주행 거리 측정을 생성하기 위해 포인트 클라우드와 feature 포인트 깊이를 연관

##### VIL SLAM
- V-LOAM 및 VINS-MONO를 기반으로 하는 LIDAR-Visual-Inertial SLAM 시스템을 제안
- 키 프레임 데이터베이스를 유지 관리하여 마일 추정(거리 단위 x마일) 및 백엔드 글로벌 포즈 그래프 최적화를 위해 V-LOAM 기반 접근 방식을 사용
  - 이 접근법에서 LO의 포즈 추정 결과는 VIO 시스템을 수정할 수 있음

##### Stereo Visual Inertial LiDAR SLAM
- 쌍안 카메라를 사용하여 LIDAR-Visual-Inertial 시스템을 형성, 이 시스템은 쌍안 기반 VIO와 LIDAR 기반 LO의 두 부분으로 나뉨
  - 쌍안 VIO 시스템은 스테레오 매칭 및 IMU 측정을 사용하여 IMU 사전 통합 합계 및 포즈 그래프의 긴밀한 결합을 수행하여 지연 프레임을 주변화하여 LO에 정확하고 안정적인 초기 포즈를 제공
  - LOAM을 기반으로 LO는 비전 기반 폐쇄 루프 감지 및 포즈 그래프 기반 백엔드 최적화를 추가하고 iSAM2를 사용하여 LIDAR 주행 거리 측정 요소 및 폐쇄 루프 요소를 점진적으로 최적화
- 이 작업은 밀접하게 결합된 시스템에 접근했지만 시스템의 VIO 및 LO는 여전히 상대적으로 독립적
- 효율적인 폐루프 감지 및 백엔드 최적화는 이러한 단점을 보완하고 나중에 등장한 다수의 tightly coupled system의 토대를 마련

##### Pronto
- LIDAR 스캔 매칭을 위해 시각적 및 열화상 이미지의 inertial prior 결과를 사용.
- 다양한 복잡한 환경에 적응하기 위해 저자는 조명이 없는 긴 터널에서 작업하기 위해 시각 및 열화상 관성 주행 거리 측정법을 사용
- leg odometry + IMU를 위한 EKF Fusion
  - 다리가 있는 로봇에게 효과적 

##### Handheld SLAM
- LiDAR-aided vision SLAM system이며 새로운 feature 깊이와 깊이 불확실성 추정 방법을 사용
- LiDAR, camera, IMU 동시 사용하여 세 가지 유형의 시각적 특징을 균일하게 매개변수화
  - 휴대용 장치에 잘 적응 

##### CamVox
- 비젼을 어시스트하는 최초의 Livox LIDAR SLAM 시스템
- LOAM-Livox와 달리 IMU는 비반복적으로 스캔된 포인트 클라우드의 왜곡 보정에 사용
- 또한 저자는 Livox LIDAR의 비반복 스캐닝 기능을 활용하여 통제되지 않은 장면에서 카메라와 LIDAR 사이의 자동 보정을 수행
- 시스템은 VINS-MONO 및 ORB-SLAM2보다 더 나은 포즈 추정 결과를 얻음
  - ORB-SLAM : ORB SLAM은 Feature-Based SLAM이고 Monocular camera기반

##### LiDAR-Visual-Inertial Estimatior
- 무결성과 견고성을 향상시키기 위해 다중 odometry를 사용
- 포인트 클라우드 기반의 localization 평가 방법과 채점 기준을 정의하여 최적의 포즈 결과를 생성
- 그러나 시스템에 데이터 연결 또는 공유가 없음
- LiDAR-Visual-Inertial Estimatior은 LIDAR의 깊이 정보를 시각적 특징에 효율적으로 할당하기 위해 복셀 맵 구조에 의존하는 LIDAR 지원 VIO 시스템을 제안
  - voxel : 3D에서 복셀은 3차원 공간에서 일반 grid의 값 
- 또한 이 작업은 이미지의 소실점 정보를 시각적 주행 거리계에 혁신적으로 도입하여 회전 드리프트를 더욱 줄임 

##### <LVI loosley coupled system 정리>
- 최근, 시스템은 포인트 클라우드와 이미지를 출력으로 사용
- pose estimation algorithm에는 ICP, NDT 를 포함한 GICP, P2P-ICP, ColorICP가 포함
- positioning은 robustness지만, loosley coupled 이기 때문에 vision과 lidar 사이의 독립으로 이어짐
  - 데이터 간의 제약 조건이 충분히 강하지 않음

## 4. Multi-Sensor Tightly Coupled System Based on LIDAR

![image](https://user-images.githubusercontent.com/108650199/193764229-44f9910a-ef01-473c-b425-83c627965545.png)

- Tightly coupled system의 경우 IMU를 다른 주행 거리계와 효과적으로 융합하는 것이 핵심 문제
- 이 작업에서는 IMU 사전 통합 공식, 오류 전달 모델 및 잔차 정의가 추론되며 이는 LIO 및 VIO의 후속 개발에 큰 영향
- 더욱이, 이러한 방정식과 모델은 Tightly coupled system의 공동 최적화를 위한 이론적 기초가 됨

- 시스템은 IMU 데이터가 추가되고 최적화되도록 IMU의 바이어스를 업데이트할 수 있음
  - 따라서 실제 값에 더 가까운 측정값이 얻어짐
  - IMU bias : IMU Bias란 Input value와 Output value의 일정한 offset을 의미 
    - IMU 센서의 경우, 매우 빠른 속도로 값(100Hz 이상)을 관찰하고 측정하기 때문에 출력으로 내보내는 값인 가속도 값(accelerometer)와 각속도 값(gyroscope)이 들어오는 시점과 이를 내보내는 시점의 일정한 차이가 있기 마련
    - Bias는 IMU 센서가 측정할 때 마다 그 값이 바뀌므로 보통 SLAM에서 최적화를 할 때 Bias 최소화하는 식으로 구함 
- 잔차를 정의하면 IMU가 다른 센서 odometry의 잔차 항을 결합하여 보다 완전한 오류 함수를 생성하는 것이 더 쉬움
- 이것은 tightly coupled의 optimization의 근거

- 👉️ 즉, Tightly coupled system은 각각의 센서들에서 나온 관측치들을 한꺼번에 하나의 cost function을 통해 optimal state를 추정하는 방법
  - e.g. Tightly-coupled VIO : 카메라 센서 관측치에 대해서는 reprojection error를 계산, IMU 센서에 대해서는 kinematic residual을 계산, 이후 이 값들을 합한 값이 최저치가 되는 motion을 추정 

#### 4-1. LI(LIDAR-IMU) Tightly Coupled System

![image](https://user-images.githubusercontent.com/108650199/193767108-eb2b0861-a5d2-4e5e-bef2-fbf5f2c6f43c.png)

##### LIPS
- LiDAR와 IMU를 tightly coupled하는 초기 접근 방식 중 하나
- 그래프 기반 최적화 프레임워크를 사용
- 이 프레임워크에서는 가장 가까운 점의 평면 표현이 제안 > 수 많은 point cloud 중 평면(Plane)을 추출하는 방법을 채택
- 포인트 클라우드 세트는 평면 기능으로 매개변수화되고, 잔차 함수는 IMU pre-integration의 잔차 항과 함께 최종 최적화 기능을 구성하는 두 프레임의 평면 매개변수 간의 차이로 변환

- [Graph-based SLAM](https://taeyoung96.github.io/slam/SLAM_04_1/)
  - 로봇은 움직이면서 자신의 pose(x, y, z, orientation 등)가 변화
  - 이렇게 움직이면서 자신의 pose를 IMU나 GPS와 같은 센서로 Tracking
  - 그와 동시에 주변환경을 카메라나 LiDAR로 인식
  - Graph-Based SLAM 에서 Pose Graph는 로봇이 움직이면서 Tracking 한 pose(Node)와 pose들 사이의 Constraint(Edge)를 저장
  
  ![image](https://user-images.githubusercontent.com/108650199/193768940-e6f75988-4060-46f1-995d-8875f4e35202.png)

  - Graph-based SLAM을 간단히 설명하면 SLAM back-end에서 Graph를 활용해서 최적화를 시키는 방법
  - 즉, SLAM system을 node와 edge의 결합으로 생각하고 이를 정교하게 풀어나아가는 것이며, 최적화를 수행할 때는 Least squares 사용
  - 요새는 다 graph slam을 사용
  - 장점 1. Observation에 대해 유연하게 대처할 수 있고, 관리하기 쉬움
  - 장점 2. Pose를 활용해서 graph를 만든다면 랜드마크(Land mark) 없이 map을 만들 수 있음

##### IN2LAMA
- IMU의 pre-integration은 raw point cloud의 왜곡을 제거하는데 사용
- IMU 및 LIDAR 데이터를 IMU의 외부 매개변수를 기반으로 하는 LIDAR 스캔의 동작을 설명하는 배치 매니폴드 최적화 공식으로 긴밀하게 통합
- 시스템은 또한 시간 차이에 대한 사전 적분 오류의 1차 형식을 고려하고 하드웨어 시간 비동기 문제를 해결

##### Lio-mapping
- LIDAR와 IMU의 측정을 공동으로 최적화하는 tightly coupled LIDAR 관성 localization 및 매핑 프레임워크인 LIOM(LIO mapping)을 제안
- 특정 규모의 최적화 데이터를 유지하기 위해 슬라이딩 윈도우 모델이 추가로 사용
- 그러나 LIOM은 모든 센서에서 측정을 처리하도록 설계되었기 때문에 실시간성 X

##### LiDAR Inertial Odometry
- 2D SLAM의 하위 그래프 표현을 3D LIDAR 주행 거리 측정에 도입하고 관성 데이터를 추가하여 프레임 간의 움직임 예측 및 제약 조건을 설정
- 이 시스템에서는 3D 점유 격자 방식을 사용하여 2D 점유 격자를 대체하여 6자유도 모두의 포즈 측정을 구현

##### LIO-SAM
- Lego-LOAM의 저자는 IMU 관련 이론과 결합하여 LIO의 후속 작업인 LIO-SAM을 발표
- 시스템은 factor 그래프에 LIDAR 관성 주행 거리 측정을 구축하고 폐쇄 루프를 포함한 여러 상대 및 절대 측정값이 시스템에 factor로 통합
- ❓️ Global map에서의 scan matching (LOAM 방식)이 아니라 old LiDAR scan을 marginalize하여 pose optimization을 수행함 ❓️
  - keyframes 의 sliding window 방식을 이용한 local scale 에서의 scan-matching 
- 시스템은 실시간 성능을 크게 향상시키기 위해 전역 일치 대신 로컬 맵 일치를 사용
- 또한 시스템은 시스템의 장기간 드리프트를 수정하는 데 사용되는 GPS 절대 위치 계수도 추가
- 그러나 특징 추출은 기하학적 환경에 의존하기 때문에 이 방법은 여전히 열린 장면에서 오랫동안 작동하지 않음

##### LIRO
- LIO와 UWB 거리 측정을 결합한 센서 융합 방식을 제안
- 이 시스템은 IMU, LIDAR 및 UWB 데이터를 슬라이딩 창에서 타임스탬프 기반 로봇 상태와 긴밀하게 연결하여 UWB, LO 및 IMU 사전 통합으로 구성된 비용 함수를 구성
- 마지막으로 factor 그래프 모델은 창 내에서 데이터를 점진적으로 주변화하고 업데이트하는 데 사용

##### Inertial Aided 3D LiDAR SLAM
- 프론트 엔드를 더욱 개선
- ground features, edge features, and plane features을 포함하여 원래 포인트 클라우드의 명시적인 혼합 특징을 동시에 추출하는 효율적인 알고리즘을 제안
- 딥 러닝 기반 LPD-Net을 도입
- 루프 폐쇄 감지는 키 프레임 데이터베이스에서 수행

##### KFS-LIO
- 시스템의 실시간 성능을 보장하기 위해 포인트 클라우드 피쳐 제약 조건에 대한 정량적 평가 방법과 주요 피쳐에 대한 스크리닝 알고리즘을 제안
- 효과적인 절충안은 정확도와 계산 비용 사이에서 절충

##### CLINS
- LIO 시스템이 고주파수 비동기 센서 데이터를 효율적으로 융합하기 위한 고정밀 연속 시간 궤적 추정 프레임워크를 제안
- 이 시스템은 연속 시간 궤적 추정을 위해 비강체 등록 방법을 사용
- 궤적 추정을 더욱 최적화하기 위해 동적 및 정적 제어점이 정의
- 동시에 폐루프 포즈와 제어점을 각각 효과적으로 업데이트하기 위해 2단계 폐루프 보정 방법을 제안
- 그러나 루프를 닫는 계산 비용은 보고되지 않으며 모션 저하로 인해 발생할 수 있는 모션의 불확실성도 다루지 않음

##### RF-LIO
- LIO-SAM을 기반으로 제안된 동적 SLAM 프레임워크
- 시스템은 포인트 클라우드로 구성된 다중 해상도 범위 이미지를 적응적으로 추가하고 밀접하게 결합된 LIDAR 관성 주행 거리계를 사용하여 움직이는 물체를 제거
- 그런 다음 LIDAR 스캔이 하위 그래프와 일치
- 따라서 매우 동적인 환경에서도 정확한 포즈 추정 결과를 얻을 수 있음

##### FAST-LIO
- 솔리드 스테이트 LIDAR를 기반으로 하는 LIO 긴밀하게 결합된 시스템
- 그러나 장시간 열린 장면에서 이동할 때 LIO 열화가 여전히 발생
- FAST-LIO는 UAV 시스템을 위해 밀접하게 결합된 반복 칼만 필터를 기반으로 하는 효율적이고 강력한 LIO 프레임워크를 제안
- 그러나 시스템은 현재 상태에 대한 기록 데이터의 영향을 무시
- 전역 포즈 보정을 수행할 수 없음

#### <LI tightly coupled system 정리>
- 관성 시스템의 긴밀한 결합은 의심할 여지 없이 정확도를 향상시키면서 시스템의 계산 부담을 증가
- 기존 알고리즘의 대부분은 과거 데이터를 주변화하거나 로컬 지도 용량을 제한하여 컴퓨팅 속도를 향상
- 백엔드 최적화는 일반적으로 IMU에서 측정한 편향과 속도를 추가하지 않고 LIDAR의 포즈 그래프만 작성
- 이러한 방법은 대부분의 시나리오에서 우수한 결과를 얻음
- 그러나 기하학적 특징에 대한 의존성으로 인해 개방된 비정형 장면에서 관성 시스템이 LO 제약 조건을 잃으면 SLAM은 심각한 드리프트 및 저하를 겪음

- IMU pre integration 이론의 개발 및 개선으로 LO 시스템은 IMU와 더 강력한 제약 관계를 설정
- SLAM 시스템의 위치 파악 정확도도 더욱 향상
- 그러나 긴밀한 결합에는 많은 계산이 필요
- 속도와 정밀도 사이의 균형을 찾는 것이 이 작업 단계의 어려움

#### 4-2. LVI(LIDAR-Visual-IMU) Tightly Coupled System
- Vision은 장면의 구조에 제약을 받지 않기 때문에 LIDAR와 훌륭한 보완 관계를 형성
- 포인트 클라우드와 이미지 간의 강력한 데이터 연관을 통해 시스템은 전처리 단계에서 여러 효과적인 기능을 긴밀하게 결합
- LVI Tightly Coupled System은 최적화와 필터링을 기반으로 두 가지 결합 방법으로 나뉨
##### 👉️ optimization-based approach
- 최적화 기반 접근 방식은 개별 센서의 오류 모델을 긴밀하게 통합하고 로컬 맵 또는 슬라이딩 윈도우를 사용하여 시간 동기화의 민감도를 줄임
- 이 방법은 역사적 포즈를 동시에 최적화하고 BA로 실시간 성능을 달성
  - 💥️ BA : Bundle Adjustment (BA)
    - 연속적인 카메라 영상 + 매칭 쌍이 주어졌을 때 카메라 포즈 전체와 월드 상 점들 전체에 대한 최적의 위치를 찾는 방법 
- 또한, 본 논문에서는 센서 데이터 레벨의 tight association을 tightly coupled된 특별 카테고리로 분류
- 포즈 해석 시 조인트 최적화는 수행되지 않지만, 이러한 작업은 전처리를 통해 데이터를 강력하게 상관시키고 이미 하나의 목적 함수에 센서에 필요한 핵심 정보를 포함
- 그들은 모두 폐쇄형 시스템을 가지고 있어 확장성이 낮고 다른 센서와 호환

##### LIMO
- 시스템은 안정적이고 강력한 시스템을 구현하기 위해 다양한 데이터 전처리 방법을 통해 포인트 클라우드와 이미지 간의 강력한 상관 관계를 수행
- 시스템은 시각적 특징의 최상의 깊이 추정을 얻기 위해 다양한 장면에 대한 포인트 클라우드에서 foreground segmentation, plane fitting, and ground fitting을 수행
- 3D-2D PNP(Perspective-n-Point-Problem)포즈 추정 방법과 2D-2D 에피폴라 제약을 결합하여 우수한 위치 파악 효과를 달성

##### LiDAR-Monocular Visual Odometry
- 이미지에서 점과 선의 특징을 추출하고 LIMO와 유사한 방법으로 점과 선의 위치 정보를 얻는 강력한 데이터 상관관계의 또 다른
- 또한 3D-3D ICP 모델 및 점과 선의 재투영 오차 함수를 구성하여 보다 정확한 포즈 추정을 달성

##### LIDAR-Monocular Surface Reconstruction
- LIDAR 스캔과 이미지 데이터에서 공동 감지된 기하학적 특징을 사용하여 상위 수준 공간에서 두 센서의 데이터를 융합
- LIDAR 스캔에서 추출된 3D 선과 이미지에서 감지된 2D 선 사이의 일치가 결정

##### DSP-SLAM 
- 객체 감지와 SLAM을 결합하여 DSP-SLAM 시스템을 제안
- 이 작업은 DeepSDF 네트워크를 사용하여 키프레임된 포인트 클라우드 및 이미지 데이터에서 객체 모양 벡터와 7D 포즈를 생성
- surface loss 및 깊이 렌더링 손실 함수를 최소화하기 위해 작은 수의 포인트 클라우드 및 이미지 분할 결과를 관찰로 사용
- 객체 재구성 및 포즈 업데이트가 ORB-SLAM2 기반 BA 팩터 그래프에 추가되어 카메라 포즈, 맵 포인트 및 객체 포즈를 동시에 최적화

##### GR-Fusion
- 카메라, IMU, LIDAR, GNSS 및 모션 섀시(차대)의 인코더를 주요 센서로 사용하여 슬라이딩 창에서 factor 그래프 모델을 구축
- LIDAR factor, 시각 factor, IMU factor, 주행 거리 측정 factor이 factor 그래프에 기본 제약 조건으로 추가
- 한편 로컬 제약 조건은 GNSS 제약 조건과 밀접하게 결합되어 로봇의 전역 상태를 최적화
- 시스템은 실시간으로 센서의 열화를 감지하고 다양한 시나리오에 적합한 다중 작업 모드를 유연하게 구성

##### Lvio-Fusion
- GR-Fusion과 비슷하지만 다른 점은 stero(쌍안) camera가 visual sensor로 사용
- 다양한 시나리오에서 센서의 가중치를 적응적으로 조정하기 위해 강화 학습 네트워크가 도입

##### LVI-SAM 
- Lego-LOAM과 LIO-SAM의 저자들의 최신 작품
- 시스템은 VIS(Visual-Inertial System)와 LIS(LIDAR-Inertial System)로 구성
- VIS는 VIS 초기화를 위한 포즈 추정 및 정확한 특징점 깊이를 제공할 수 있는 LIS에 대해 포즈 이전을 제공
- 그러나 이 시스템은 LIO 시스템의 주변화 및 타임스탬프 동기화 문제를 고려하지 않음

##### Super Odometry

> Factor graph structure of Super Odometry

![image](https://user-images.githubusercontent.com/108650199/193791828-cf65ebb8-01bf-4f66-a35d-91fd47c83c2b.png)

- 일부 주행 거리 측정 시스템은 낮은 드리프트 포즈 추정치를 얻기 위해 밀접하게 결합된 접근 방식을 사용
  - 그게 바로 Super Odometry 
- Super Odometry는 IMU를 메인 센서로 하는 LVI 시스템을 사용
- 시스템은 IMU 주행 거리계, VIO 및 LIO의 세 부분으로 구성
- VIO 및 LIO에서 제공하는 관찰 데이터는 IMU의 편향을 제한
- 반면에 제한된 IMU 주행 거리 측정은 VIO 및 LIO가 대략적인 자세에서 미세한 자세 추정을 달성하도록 예측을 제공
- 시스템은 센서 성능 저하에 대한 견고함으로 GPS와 휠 주행 거리계를 동시에 확장할 수 있음

##### Tightly Coupled LVI Odometry
- LIDAR와 쌍안 카메라를 기반으로 하는 긴밀하게 결합된 시스템을 제안
- factor 그래프 최적화 문제는 initial prior factor, visual factor, line factor, plane factor, and IMU pre-integration 및 IMU pre-integration으로 구성되며 GTSAM으로 해결
- 시스템은 비전 기반 포인트 피처와 포인트 클라우드 기반 라인 및 영역 피처의 표현을 공유
- 재투영 오류 함수는 다른 기능을 매개변수화하여 정의
- 또한 시스템은 카메라의 데이터 타임스탬프를 기반으로 LIDAR의 인접 스캔 데이터를 분할 및 병합하여 소프트웨어 수준에서 시간 하드 동기화를 실현 
- 이와 같은 방법은 실시간 포지셔닝 효과가 좋음
- but, 순수 주행 기록계 시스템은 종종 과거 데이터를 버리고 현재 또는 지역 관측에만 초점을 맞추므로 전체 데이터의 상관 관계 및 최적화가 손실
- 🌟️ 따라서 SLAM 시스템에는 우수한 프론트 엔드 주행 거리계와 합리적인 백엔드 최적화가 필요 🌟️

##### 👉️ Filter-based approach
- 현재 프레임의 센서 데이터만 사용하고 각 데이터의 시간 동기화에 의존
- 현재 포즈에 대한 과거 데이터의 영향을 고려하지 않았기 때문에 계산량이 상대적으로 적고 확장성이 상대적으로 좋음

##### Tightly-coupled aided inertial navigation
- MSCKF를 사용하여 RGB-D 센서의 평면 형상, IMU 측정 및 3.5m 이내의 시각적 포인트 형상을 밀접하게 결합
- 상태 벡터의 크기를 제한하기 위해 시스템은 대부분의 포인트 기능을 선형으로 주변화하고 상태 벡터에 평면 강화 제약 조건이 있는 몇 가지 포인트 기능을 SLAM 기능으로 유지

##### LIC-Fusion 
- LIC-Fusion은 IMU 측정, 추출된 LIDAR edge features 및 sparse visual features을 밀접하게 결합하는 MSCKF 융합 프레임워크를 채택
- 이후, 3D LIDAR 포인트 클라우드를 실시간으로 효율적으로 처리하기 위해 슬라이딩 윈도우 기반 planar feature tracking method을 도입

##### R2LIVE
- 솔리드 스테이트 LIDAR를 기반으로 한 긴밀하게 결합된 작업
- 시스템은 error-state-based iterative Kalman filtering front end와 새로운 단계의 factor graph optimization-based sliding window optimization를 결합하여 시각적 포즈와 랜드마크 추정을 개선
- 실내, 실외, 터널 및 고속 모션과 같은 가혹한 시나리오에서 높은 정확도와 견고성을 달성
- 이러한 방법은 빠르고 계산 비용이 저렴하지만 시간 동기화에 민감
- 따라서 서로 다른 센서의 측정 결과의 정확한 순서를 보장하기 위해 특별한 분류 메커니즘이 필요

#### <LVI tightly coupled system 정리>
- LVI tightly coupled system은 strong data correlation, nonlinear optimization tight coupling, and state filter의 세 부분으로 나뉨
- 그 중 최적화를 기반으로 tightly coupled front-end가 주요 구현
- 뿐만 아니라 wheel/leg odometry 및 GNSS도 시스템에 효과적으로 통합

## 5. 평가
- SLAM 알고리즘의 평가는 주로 위치 정확도 평가를 기반
- 상대 포즈 오차(RPE)는 특정 시간 차이로 분리된 두 프레임 간의 포즈 차이의 정확도를 설명하는 데 사용
- 실제 포즈와 추정 포즈의 변화는 동일한 시간 간격으로 계산
- 그런 다음 두 값의 차이를 계산하여 상대 포즈 오차를 구함
- 이후 각 기간의 상대 자세 오차를 RMSE(Root Mean Square Error)로 계산하여 전체 값을 얻음
