Visual-SLAM Classical Framework and Key Techniques: A Review
===

## 3. V-SLAM Classical Framework

![image](https://user-images.githubusercontent.com/108650199/194191510-9a65add4-2728-4bff-a3e0-0665b8cc21fc.png)

### 3.1 Front-end

![image](https://user-images.githubusercontent.com/108650199/194000248-6ddb2fee-1636-49e0-893f-c18884d29d7a.png)

- 현재 프레임의 위치를 결정하기 위해 카메라가 움직이는 동안 입력 이미지를 처리하고 모션 관계를 얻는 것

#### 3.1.1 Visual sensor
- visual sensor : 카메라 이미지 정보를 읽고 전처리하는 역할
  - Mono 구조가 간단하고 계산 속도가 빠르지만 정보의 깊이가 부족하고 스케일 블러
  - Stereo 실내외의 깊이 정보를 얻을 수 있지만, 필요한 연산량은 상당
  - RGB-D 이미지 색상과 깊이 정보를 동시에 얻지만, TOF를 이용해서 빛에 간섭 & 측정 범위에 취약

#### 3.1.2 Visual Odometry
- visual odometer : 백엔드에 더 나은 초기 값을 제공하기 위해 인접 이미지의 데이터를 기반으로 카메라 움직임을 추정
  - Featured based와 Direct based로 구분
- VI-SLAM(Visual Inertial SLAM)은 복잡한 동적 장면에서 포즈 정확도와 적응성의 장애물을 더 잘 해결하기 위한 연구 및 적용에서 선호
  - V-SLAM과 IMU(관성 측정 장치)간의 보완  
  - SLAM은 IMU가 누적 오류를 제거하고 폐쇄 루프 감지를 완료하는 데 사용
  - IMU는 더 적은 질감과 빠른 움직임으로 위치 정확도를 해결하는 SLAM을 지원하는 데 사용
- VO에 의한 경로 재구성은 특징점 방식과 직접 방식을 사용

### 3.2 Backend Optimization
- 프론트엔드에서 제공하는 시각 센서에서 수집한 원본 데이터를 받아 계산 및 최적화를 수행
- 동시에 백엔드 최적화는 본질적으로 필터 기반 또는 비선형 최적화 방법을 사용하는 상태 추정 문제
- 이전에는 💥️ 필터 기반 방법 💥️이 V-SLAM의 백엔드에서 주요 방법
  - 프론트엔드 데이터를 최적화하고 처리할 수 있음
  - 그러나 오류가 발생하고 알고리즘의 실시간 성능을 보장할 수 없음
  - V-SLAM의 경우 시스템이 실행되는 동안 맵 포인트 및 위치 수가 증가
  - EKF가 유지하고 업데이트해야 하는 공분산 척도와 평균도 더욱 광범위
  - 한편, motion과 observation 사이의 선형 근사는 작은 범위에서 작동하고 더 먼 거리에서는 심각한 비선형 오류가 발생
- 비선형, 비가우스 시스템에 대한 EKF 알고리즘의 한계로 인해 연구자들은 PF(Particle Filter)방법도 제안 
  - PF 방법의 장점은 상태 추정이 데이터 연관에 민감하지 않지만 선형 근사에서 더 나은 성능을 갖는다는 것 

- 디지털 영상 처리 기술의 발전으로 필터 기반 기술은 점차 💥️ Graph based optimization 💥️ 로 대체

### 3.3 Loop Detection
- 오류를 correction하는데 사용
- 인접 프레임을 제외한 다른 프레임에 제약을 추가할 수 있으며 위치 지정 및 매핑과 밀접한 관련
- 백엔드는 프론트엔드에서 제공하는 데이터를 최적화할 때 최대 오류를 추정하고 루프 감지는 💥️ 오류 누적 💥️으로 인한 영향을 제거
- mono camera에 대한 세 가지 loop detection 방법 제시
  - 1) image-to-image
    - 표준 문자열 어휘에서 얻은 그래픽 기능을 사용하여 두 문자열 사이의 동일한 위치를 감지
    - 최신 이미지와 이전 이미지 사이의 일관된 지점을 결정 
    - 🌟️ 이미지 대 이미지 방법이 가장 높은 정확도 🌟️
  - 2) map-to-map 
    - 두 개의 서브맵에서 ID 간의 대응 관계를 찾음
    - 매칭 결과는 두 이미지 사이에 공통된 ID를 찾을 수 있지만 공통된 feature의 수는 충분하지 않음을 나타냄
  - 3) image-to-map
    - 최신 프레임과 맵 사이에 유사한 feature를 찾아야함
    - 3점 위치 계산을 통해 카메라의 위치와 다른 물체의 위치를 판단

#### - [Bag of Words](https://darkpgmr.tistory.com/125)
- loop detection에서 "Bag of Words"(BoW)는 광범위하게 적용
- Bag of Words란 단어들의 순서는 전혀 고려하지 않고, 단어들의 출현 빈도(frequency)에만 집중하는 텍스트 데이터의 수치화 표현 방법
- 영상처리, 컴퓨터 비전 쪽에서는 Bag of Words 기법을 주로 이미지를 분류(image categorization)하거나 검색(image retrieval)하기 위한 목적으로 사용
- 최근에는 물체나 씬(scene)을 인식하기 용도로도 폭넓게 활용

![image](https://user-images.githubusercontent.com/108650199/194192022-ccc5b543-976f-42a8-bf40-250c895b16f5.png)

##### - How to Bag of Words ?
- 영상 분류를 위한 Bag of Words 방법은 먼저 영상에서 feature(주로 SIFT 등의 local feature)들을 뽑은 후, 이들 feature들을 대표할 수 있는 값(code)들로 구성되는 코드북을 생성
- 보통 코드북은 다수의 이미지들로부터 추출한 feature들 전체에 대해 클러스터링(k-means clustering)을 수행하여 획득한 대표 feature(각 cluster의 center)들로 구성
- 이 코드북은 일종의 단어 사전(dictionary)으로 볼 수 있음
- 이 사전에는 가능한 모든 종류의 단어들이 포함되어 있는 것이 아니라 물체나 이미지를 분류하는데 있어서 중요하다고 생각되는 주요 단어들만이 포함되어 있는 점이 다름
- 코드북에 포함된 단어를 코드워드(codeword)라 부르는데, 코드북을 몇 개의 codeword로 구성할 지는 조절 가능한 파라미터로서 영상 feature들을 몇개의 클러스터로 클러스터링할지에 따라 결정
- 일단 코드북이 완성되면 이제 각각의 이미지들을 이 코드북을 이용하여 표현(representation)할 수 있음
- 먼저 A로부터 feature들을 추출한 후 추출된 각각의 feature들에 대해 코드북 내에서 대응되는(가장 유사한) 코드워드(codeword)를 찾음
- 이렇게 찾은 코드워드들의 히스토그램(histogram)으로 이 이미지의 특징을 표현
- 이렇게 구한 코드워드의 히스토그램(각각의 코드워드가 이미지에서 몇번 나타났는지 개수를 센 것)이 동일 종류의 물체에 대한 이미지들 사이에서는 유사하고 다른 종류의 물체에 대해서는 서로 다를 것이라는 것이 Bag of Words를 이용한 이미지 분류의 핵심 아이디어

![image](https://user-images.githubusercontent.com/108650199/194192745-190837fa-090e-45f3-b6e0-69cb94d2c975.png)

- 이상의 Bag of Words 방법을 이용한 이미지 분류 과정을 정리해 보면 다음과 같음
- 1) Feature Extraction: 이미지들로부터 feature를 추출한다 (SIFT 등)
- 2) Clustering: 추출된 feature들에 대해 클러스터링을 수행하여 클러스터 센터(center)들인 codeword들을 찾아냄
  - 클러스터링 방법은 보통 k-means clustering이 사용
- 3) Codebook Generation: 찾아진 codeword들로 구성되는 코드북을 생성
  - 코드북은 물체 클래스마다 생성하는 것이 아니라 공통적으로 하나만 생성되며 모든 클래스의 codeword들을 포함
- 4) Image Representation: 각각의 이미지들을 codeword들의 히스토그램으로 표현
  - 이미지 하나당 하나의 히스토그램이 나오며 이 히스토그램의 크기(bin의 개수)는 코드북의 크기(코드북을 구성하는 codeword들의 개수)와 동일
- 5) Learning and Recognition: BoW 기반의 학습 및 인식 방법은 크게 Bayesian 확률을 이용한 generative 방법과 SVM 등의 분류기를 이용한 discriminative 방법이 있음
  - Bayesian 방법은 물체 클래스별 히스토그램 값을 확률로서 해석하여 물체를 분류하는 것이고, discriminative 방법은 히스토그램 값을 feature vector로 해석하여 SVM(support vector machine)등의 분류기에 넣고 클래스 경계를 학습시키는 방법

### 3.4 Mapping 
- 매핑은 보다 정확한 위치 지정, 탐색, 장애물 회피, 재구성 및 상호 작용을 위한 것
- 프론트엔드 최적화이든 백엔드 최적화이든 최적화는 매핑을 위한 적절한 준비를 함
##### - Mono-SLAM
- 백엔드로 EKF를 사용하는 최초의 실시간 monocular visual system을 제안
- 가장 중요한 이점은 실시간 이미지와 드리프트가 없다는 것
- 단안 카메라는 깊이 정보를 얻을 수 없기 때문에 시스템 시작을 돕기 위해 소량의 이전 장면 정보를 사용
- 루프가 닫히기 전에 불확실성이 증가했으며 드리프트 수정이 이루어짐
- 이 방법의 단점은 장면이 좁고 랜드마크의 수가 제한되어 있으며 희소한 feature이 손실되기 쉬움
##### - PTAM

> PTAM processing diagram

![image](https://user-images.githubusercontent.com/108650199/194198314-9a2a145f-05ae-42b4-b792-ca940ae78b5a.png)

- 2 thread 구조
  - 비교적 연산로드가 적은 카메라 tracking은 모든 영상프레임에 적용하여 실시간성을 추구
  - 맵 갱신은 주요 키프레임에만 적용하되 시간이 오래 걸리더라도 정밀한 알고리즘을 사용하여 정확도를 추구
- nonlinear optimization backend solution을 최초로 사용하여 V-SLAM의 백엔드 처리가 비선형 최적화에 의해 지배될 수 있는 기반을 마련
- PTAM은 또한 키 프레임 메커니즘을 제안
  - key frame : 변화가 큰 장면이나 일정 시간 간격으로만 전체 이미지를 저장하고 그 사이의 장면은 변화되는 정보만 저장하는 비디오 압축 기법 
  - 이 때, 전체 이미지가 저장되는 영상프레임을 키프레임
- 각 이미지를 신중하게 처리하는 대신 여러 주요 이미지를 연결하여 궤적과 지도를 최적화
- PTAM은 가상 평면에 가상 물체를 배치할 수 있으며 AR(증강 현실)과 SLAM을 결합하는 데 기여

##### - ORB-SLAM
- 3 스레드 구조를 제안
- sparse map의 구성을 실현할 수 있지만 위치 요구만 충족할 수 있음
  - navigation, obstacle avoidance 또는 기타 기능을 제공할 수 없음
- ORB-SLAM에 사용된 시각 센서는 단안 카메라로 스케일 드리프트 문제 
##### - ORB-SLAM2
- ORB-SLAM의 단점을 기반으로 ORB-SLAM2는 단안, 스테레오 및 RGB-D 카메라를 위한 최초의 SLAM 시스템으로 2016년에 제안
- global BA(번들 조정)를 실행하기 위해 루프 감지에 영향을 미치지 않도록 스레드가 설정
- VO를 사용하여 매핑되지 않은 영역을 추적하고 제로 드리프트 포지셔닝을 달성하기 위해 맵 포인트를 일치시키는 경량 포지셔닝 모델이 포함
##### - LSD-SLAM
- 키프레임 간의 유사성 변환을 direct 추정하는 이미지 매칭 알고리즘을 제안
- 이미지의 특징 기술자를 추출할 필요가 없으며 광학 측정 오차를 최적화하여 두 프레임 간의 변환을 얻을 수 있음
- 최종 결과는 텍스처가 약한 곳에서 더 잘 작동하는 semi-dense map
- LSD-SLAM의 제안은 sparse 맵에서 semi-dense 맵으로의 전환을 표시
##### - SVO-SLAM
- 많은 수의 디스크립터를 계산하지 않고 semi-direct visual odometry를 사용하며 매우 빠름
- laptop에서는 초당 300프레임, UAV(무인항공기)에서는 초당 55프레임
- 임계점의 위치를 추정하고 inverse depth를 매개변수화된 형식으로 사용하기 위해 깊이 필터의 개념을 먼저 제안
- 단점은 백엔드 최적화 및 루프 감지를 버리고 위치 추정에 누적 오류가 있으며 손실 후 재배치가 어렵다는 것
##### - DSO-SLAM

![image](https://user-images.githubusercontent.com/108650199/194200608-bfec792c-a0ff-4894-b794-1990ba5d9f46.png)

- 정확도, 안정성 및 속도 측면에서 LSD-SLAM보다 우수
- prior information를 고려하지 않고 측광 오차를 직접 최적화
- 최적화 범위는 모든 프레임이 아니라 가장 최근 프레임과 이전 몇 프레임으로 구성된 슬라이딩 윈도우
- DSO-SLAM은 직접 방법 위치 추정의 오류 모델을 완성하는 것 외에도 아핀 밝기 변환, 측광 보정 및 깊이 최적화를 추가
- 그러나 이 방법에는 루프 감지 기능이 없음
- 삽입된 빨간색 선은 추적된 궤적과 함께 누적된 드리프트를 시각화하는 시작 및 끝 위치 주기를 보여줌

#### <정리>
- 매핑 개발은 Mono-SLAM, PTAM, ORB-SLAM, LSD-SLAM, SVO-SLAM 및 DSO-SLAM과 같은 다양한 V-SLAM 알고리즘을 거쳤음
- 매핑의 정확도, 안정성 및 속도가 향상
- 각각의 단점이 있기 때문에 응용 환경에 따라 적절한 매핑 기술을 선택하는 것이 필요

## 4. V-SLAM Key Techniques
### 4.1 Feature Detection and Matching
- V-SLAM 프론트엔드의 주요 목적은 백엔드에 더 나은 데이터를 제공하는 것이며, 여기서 visual odometer가 핵심적인 역할
- 카메라 이미지를 분석하여 로봇의 위치와 방향을 결정
- 그리고 특징을 추출해야 하는지 여부에 따라 특징점 방식과 직접 방식으로 나눌 수 있음
- 특징점은 높은 안정성 때문에 가장 일반적으로 사용
- 사진 및 기타 원본 정보는 연속 아날로그 신호입
- 따라서 컴퓨터가 데이터를 처리할 수 있도록 디지털 형식으로 변환해
- 디지털 이미지는 종종 그레이 매트릭스 형태로 저장되지만 그레이 값은 조명 요인과 물체의 재질에 따라 변함
- 임계점의 식별 및 위치를 더 자세히 연구하려면 이미지에서 관점의 변경으로 변경되지 않는 점, 즉 특징점을 선택하는 것이 필요
- 일부 장면에서는 이미지의 특징점이 요구 사항을 충족하지 않음
- 따라서 보다 안정적인 인간이 설계한 특징점이 생성


![image](https://user-images.githubusercontent.com/108650199/194227628-1d7911c4-b735-442f-b391-1602910b9ca0.png)

##### - feature detection
- 영상에 관심있는 Feature(Edge, Corner) 등을 검출
- 이러한 feature 중 좋은 영상 특징점(keypoint)가 되기 위한 조건은 아래와 같음
  - 물체나 형태나 크기, 위치가 변해도 쉽게 식별이 가능한 것
  - 카메라의 시점, 조명이 변해도 영상에서 해당 지점을 쉽게 찾아낼 수 있을 것
- ex ) 👉️ keypoint detector : FAST (Harris corner 방식보다 20배 빠름)
##### - descriptor
- 특징점 주변 픽셀을 일정한 크기의 블록으로 나누어 각 블록에 속한 픽셀의 그레디언트 히스토그램을 계산한 것
- detection된 feature 주위의 밝기, 색상, gradient 방향 등의 matching 정보를 계산
-  descriptor는 keypoint에 해당하는 정보이므로, 기본적으로 keypoint와 같은 개수로 생성되며 실제 유사도를 판별하기 위한 데이터로 활용
- 일반적으로 특징점 주변의 블록 크기에 8방향(상, 하, 좌, 우 및 네 방향의 대각선) 경사도를 표현하는 경우가 많음
- 4 x 4 크기의 블록인 경우 한 개의 특징점당 4 x 4 x 8 = 128개의 값을 가짐
  - ex ) 👉️ Keypoint detector + Descriptor : SIFT, SURF, ORB 
#### 4.1.1 SIFT

![image](https://user-images.githubusercontent.com/108650199/194214186-62c4305b-cdb6-464f-9974-b7c36eb2ed01.png)

- SIFT(Scale Invariant Feature Transform)는 이미지 회전 및 크기 불변성을 가진 고전적인 특징점
  - 서로 다른 두 이미지에서 SIFT 특징을 각각 추출한 다음에 서로 가장 비슷한 특징끼리 매칭해주면 두 이미지에서 대응되는 부분을 찾을 수 있다는 것이 기본 원리
  - 크기와 회전은 다르지만 일치하는 내용을 갖고 이미지에서 동일한 물체를 찾아서 매칭해줄 수 있는 알고리즘
- 또한 이미지 노이즈와 가시 조명 변화에 대한 robustness을 가짐
  - visible illumination : 가시광선으로 인간의 밝기와 색상 인식을 유발하는 파장 범위
- 그러나 특징점의 차원이 크기 때문에 실시간으로 정확한 계산을 완료하기가 어렵고 위치 지정 및 매핑 속도가 훨씬 느려짐
  - 또한, 크기 변화에 따른 특징 검출 문제를 해결하기 위해 이미지 피라미드를 사용해서 속도 느려짐 
#### 4.1.2 SURF
- SIFT 알고리즘을 기반으로 제안되었으며, SIFT 알고리즘의 연산 속도가 느리고 연산량이 많다는 단점을 주로 개선
  - 이미지 피라미드 대신 필터의 크기를 변화시키는 방식으로 성능 개선 
- 여러 개의 영상으로부터 스케일, 조명, 시점 등의 환경변화를 고려하여 환경변화에 불변하는 특징점을 찾는 알고리즘
- SURF는 그레이 공간상의 정보만 이용함에 따라 컬러 공간상에 주어진 많은 유용한 특징들을 활용하지 못함
- SIFT 알고리즘은 이미지 크기를 변경하고 가우스 함수를 사용하여 하위 레이어를 반복적으로 매끄럽게 함 > 이미지 피라미드 O
- SURF 알고리즘은 필터 크기만 변경하고 다운샘플링 프로세스를 생략하며 처리 속도를 향상 > 이미지 피라미드 X

![image](https://user-images.githubusercontent.com/108650199/194217996-7c08c1a7-1aed-49ba-a853-5b88ad524d51.png)

#### 4.1.3 ORB
- FAST keypoint detector와 BRIEF descriptor가 합쳐짐
##### - FAST

![image](https://user-images.githubusercontent.com/108650199/194218750-53e3fc0f-6f0d-4648-b38f-8b6b6750dd73.png)

  - FAST는 영상에서 실시간으로 keypoint를 찾아내는 알고리즘으로, 하나의 keypoint에 여러 개의 feature를 가지는 SIFT와는 달리 하나의 keypoint에 하나의 feature만을 가짐
  - FAST의 핵심은 중심 픽셀과 그 주변의 원형 내에 위치한 픽셀들과의 차이를 구분
  - 이때 FAST는 자체적으로 corner를 만들어내지 않기에 Harris corner 방식으로 얻어진 keypoint를 사용하며, N개의 keypoint를 얻기 위해 임계값을 일부러 낮게 설정한 뒤 상위 N개의 keypoint만을 추출해 사용
  - 픽셀 P의 주변 픽셀에 임곗값을 적용해 어두운 픽셀, 밝은 픽셀, 유사한 픽셀로 분류해 원 위의 픽셀이 연속적으로 어둡거나 밝아야 하며 이 연속성이 절반 이상이 돼야 우수한 특징점을 갖는다고 함
  - multi-scale 기능도 지원하지 않아 scale pyramid 방식으로 각각의 단계마다 FAST feature를 생성
  - 이때 FAST에는 방향에 대한 계산이 들어가지 않아 방향에 대한 detection을 지원하지 않는다는 단점
  - 이에 intensity centroid 방법을 사용해 방향성을 얻고자함
    - 조금 더 구체적으로 설명하자면 위의 방식을 통해 구해진 코너는 중심의 픽셀값과 차이를 보이기에 중심으로부터 코너까지의 벡터는 해당 keypoint의 방향성을 설명 
##### - BRIEF
  - 특징점 검출은 지원하지 않는 디스크립터(특징점에 대한 기술자) 추출기
    - 기술자(Descriptor)란 서로 다른 이미지에서 특징점(Key Point)이 어떤 연관성을 가졌는지 구분하게 하는 역할 
  - SIFT의 연산 속도가 느린 이유는 SIFT 알고리즘이 128차원이라는 높은 차원의 벡터를 가지고 있기 때문
  - 이를 개선하기 위해 SURF가 개발되었지만, 눈에 띄는 용량의 감소는 아님
  - 이는 descriptor 자체가 많은 비트를 가지고 있어야 했기 때문
  - descriptor를 이진화(binarize)하는 방식이었고, 이렇게 나온 알고리즘이 BRIEF descriptor
    - 특징점 주변 영역의 픽셀을 다른 픽셀과 비교해 어느 부분이 더 밝은지를 찾아 이진 형식으로 저장
  - 이렇게 만들어진 BRIEF descriptor는 시점, 조명, 블러에 강인한 모습을 보임
  - 그러나 BRIEF descriptor는 이미지가 조금만 회전하더라도 성능이 확연하게 저하되는 단점

##### - ORB
  - 디스크립터 검출기 중 BRIEF(Binary Robust Independent Elementary Features)라는 것이 있음
  - BRIEF는 특징점 검출은 지원하지 않는 디스크립터 추출기
  - 이 BRIEF에 방향과 회전을 고려하도록 개선한 알고리즘이 바로 ORB
  - 이 알고리즘은 특징점 검출 알고리즘으로 FAST를 사용하고 회전과 방향을 고려하도록 개선했으며 속도도 빨라 SIFT와 SURF의 좋은 대안으로 사용

### 4.2 Selection of Keyframes

![image](https://user-images.githubusercontent.com/108650199/194221672-83d98587-e2c6-4d51-9dd2-526afd501546.png)

![image](https://user-images.githubusercontent.com/108650199/194221712-76e2959d-3118-4888-a02d-6750ff9a1702.png)

### 4.3 Uncertainty Technology
- perception 정보의 불확실성은 reference object을 차단하는 것과 같은 불완전함에서 비롯될 수 있음
- 모바일 로봇 롤러 미끄러짐, 센서 매개변수(예: 해상도) 또는 관찰 노이즈와 같은 알 수 없는 외부 힘의 영향과 같이 무작위로 인해 발생하는 불확실성에서 비롯될 수 있음
- 모바일 로봇은 환경을 알 수 없을 때 정보를 얻기 위해 센서에 의존해야 하므로 지각적 세부 사항의 불확실성은 환경 모델의 부정확성을 초래
- 센서의 측정 오차는 측정 정확도와 측정 조건 및 시간과 관련
- 센서 관찰 데이터의 불확실성은 고해상도 또는 다중 센서 데이터 융합을 채택하여 문제를 해결하고 오류 누적을 최소화
  - ex) VO 에 IMU 추가 
  - ex) VO 에 GPS 추가
- 센서마다 특성과 장점이 다르기 때문에 다중 센서 융합을 채택하면 부정적인 영향을 줄일 수 있음

### 4.4. Expression of Maps
- 모바일 로봇은 센서를 통해 주변 환경을 감지하고 결국 환경 지도를 구축
- 지도 작성은 주로 위치 지정을 수행하기 때문에 다양한 환경 요구 사항에 따라 지도를 작성
- 환경 맵의 구성 방법에는 크게 그리드 맵, 토폴로지 맵, 옥트리로 나눠짐
##### - Grip map
- 2차원 그리드 맵은 경로 계획 및 실시간 장애물 회피와 같은 모바일 로봇의 내비게이션 분야에서 널리 사용
- RGB-D 카메라는 실시간으로 장면의 3D 포인트 클라우드를 획득하고 깊이 정보를 사용하여 로컬 그리드 맵을 설정
##### -Topology map
- 환경을 그래프 모델로 추상화하는 래스터(raster) 맵을 기반
- 토폴리지 : 외형적인 연결 모양을 뜻함

> [raster](https://mapschool.io/index.kr.html)

![image](https://user-images.githubusercontent.com/108650199/194224007-d2c2ba04-a7f1-432c-83e4-ff048b8a3de6.png)

- 래스터 방식의 데이터는 디지털 카메라로 찍는 사진을 생각해보면 이해하기 쉬움
- 단순하게 생각하면, 이렇게 찍힌 사진이란 색깔이 있는 점들의 집합
##### - Octo map

> 여러 다른 해상도에서 활성 복셀의 옥트리 맵을 쿼리하는 예

![image](https://user-images.githubusercontent.com/108650199/194224811-614b096a-144c-4eff-bdc1-aa4a1eac6805.png)

- 유연하고 압축되며 지속적으로 업데이트되는 맵
- 각 옥트리 노드는 복셀이라고 하는 3차 볼륨의 공간을 나타냄
- 블록은 주어진 최소 복셀 크기에 도달할 때까지 8개의 하위 블록으로 재귀적으로 세분화
- 최소 복셀 크기는 옥트리의 해상도를 결정

- 환경 지도 유형에는 주로 그리드, 토폴로지 및 옥트리가 포함
- V-SLAM은 또한 다양한 장면 유형과 센서 유형을 기반으로 최종 지도의 종류를 선택적으로 결정
