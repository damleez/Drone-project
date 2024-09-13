ICP(Iterative Closest Point)
===
## Point Cloud Registration
- Point Cloud Registration이란 두 Point Cloud를 정렬을 하는 공간 변환을 찾는 과정
 - 이는 Mapping을 할 때 매우 중요한 과정인데 Scan matching을 하거나 Scan registration을 진행할 때 동일한 reference frame에서 보았을 때 각각의 Map point들이 일치해야 정확한 주위 환경을 Mapping 할 수 있기 때문
- 따라서 Point Cloud Registration을 통해 가장 정렬을 잘하는 💥️ Rotation Matrix R과 Translation Vector t 💥️를 찾는 것이 핵심
- 상황에 따라 해결법도 달라지는데,
### 👉️ 두 Point Cloud의 대응관계(Correspondences)를 알 때 (Known Data Association)
 → SVD algorithm 활용해 Rotation Matrix R 과 Translation Vector t를 찾음
### 👉️ 두 Point Cloud의 대응관계(Correspondences)를 모를 때 (Unknown Data Association)
 → ICP algorithm
### 👉️ Robust한 Least Squares Approaches를 이용하는 방법
 → Least Squares (요새는 point to plane이 성능 좋아서 이걸 least square해서 r과 t를 찾음)
 - Least squares의 장점
   - SVD solution은 point-to-point correspondences를 가정
   - Error function이 복잡해지면 복잡해질 수록, Least squares 접근 방법이 요구
   - 불확실성(Uncertainties)에 대해서 더 좋은 결과 (Robust)
   - Alignment를 진행할 때 서로 다른 Weight를 활용하여 Alignment 성능 좋아짐

- 즉, 결국 구해야하는건 Rotation matrix R과 Translation Vector T를 찾는 것
- svd나 least squares나 robust하게 만들려면 결국 data association을 잘해야함


## ICP(Iterative Closest Point)

![image](https://user-images.githubusercontent.com/108650199/193006852-935f8e85-9b9d-4efd-900e-9301970e9d4c.png)

- 정의 1. 기존의 데이터셋에 현재 데이터를 '정합 '시키는 방법중의 하나로, 각 데이터들의 가장 가까운점을 이용하여 연관성을 찾고 그에 맞게 현재데이터를 이동 및 회전을 시켜 기존데이터셋에 추가하는 방법 (두 시점의 센서 데이터가 잘 얼라인(Align) 되도록 transform을 구하는 것)
- 정의 2. ICP Algorithm이란 Point Cloud Registration을 진행할 때 Data Association에 대해서 모를 경우, 대응관계를 추정하고 그 대응관계를 통해 두 Point Cloud를 Align을 하는 과정 중 하나
 - 대응관계를 만들 때는 가장 가까운 점을 활용하며 반복적인 과정을 통해 오차를 최소화

### ICP 성능에 영향을 주는 [요인](https://taeyoung96.github.io/slam/SLAM_03_2/)
- ICP 성능이란 가중치를 어디에 두냐에 따라 다르겠지만 일반적으로 속도, 정확성에 가중치를 둔 성능을 얘기
#### 1. Sampling
 - 기존의 데어티셋과 현재 데이터값을 비교하여 그들의 연관성을 찾음
 - 그럼 비교할 데이터를 어떻게 선정하느냐가 중요해짐 
  - 예를 들자면, 전체데이터(All) , 일정간격(Uniform)을 둔 데이터, 랜덤(Random) 데이터 등등

#### 2. Matching
 - 데이터가 선정되었다면 그들의 어떤기준을 가지고 매칭 시킬지가 중요
 - 가까운 점(Point to point)을 매칭할지, 법선벡터(Normal)에 가까운점을 매칭할지 등등
  ```
  EXAMPLE (Data Association 이라고 볼 수 있음)
  point to point /point to plane
  closet point
  closet compatible point
  normal shooting 
  Projection-based approaches
  ```
  - 이와 같이, 매칭 대상이아닌 매칭 방법으로는 KD-TREE가 있음
  #### 👉️ KD-TREE

  ![image](https://user-images.githubusercontent.com/108650199/193011029-c3e9d279-b0b3-43cd-8213-5e88fcb62d6d.png)

  - a(7,2), b(5,4), c(9,6), d(4,7), e(8,1) 이 순으로 데이터가 트리가 삽입된다 하자
  - a는 트리가 비어 있으므로 루트노드
  - a의 x좌표를 비교하여 작은 애들은 왼쪽으로 큰 애들은 오른쪽으로 분류
  - 따라서 b는 a의 왼쪽 노드, c는 오른쪽 노드
  - d의 x좌표는 a의 x좌표보다 작으므로 왼쪽으로 가지만 이미 b가 있으므로 이번에는 y좌표를 비교
  - b의 y좌표보다 d의 y좌표가 크므로 오른쪽에 위치한다. 동일하게 e도 배치하면 위 그림과 같이 트리가 구성

#### 3. Weighting
 - 두 개의 데이터셋이 쌍을 만들었다면 그들에게 어떻게 가중치를 줄것인가가 중요
 - 모두 같은 가중치(Constant)를 줄것인가, 거리에따라 가중치를 줄것인가, 법선벡터에따라 줄건인가 등등

#### 4. Rejecting
 - 갖고 있는 데이터가 outlier라면 제거
  - 예를 들자면, 일정거리 이상이면 버리던가, 정렬을 통해 몇%를버리던가 등등


