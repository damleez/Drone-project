GRAPH BASED SLAM
===

## 0. 개요

![image](https://user-images.githubusercontent.com/108650199/193958077-8e423e40-961b-4371-b804-372764672168.png)

- Modeling > Graph based slam이 filter based slam과 가장 큰 차이점
  - 문제를 표현하기 위해 그래프를 사용 
    - SLAM문제를 확률 그래프 모델로(Probabilistic Graphical Model) 모델링
  - Node : pose of robot
  - Edge : 2 poses 사이의 공간적 제약(transformation)
    - Node와 Edge를 확률 그래프 모델로 모델링 할 시, 그래프상의 Edge로 모든 Pose들 간의 연결관계를 표현할 수 있음
    - 모든 pose들의 값에 접근이 가능 
    - 예를 들면, 같은 자리에 와서 loop closed 할 때 전체적으로 pose를 update가능
    - filter based slam은 시간에 따라 오차가 쌓여가는 문제를 이러한 global optimization으로 해결 가능
- Front-end & Back-end
  - Front-end : 각 time-stamp마다 pose를 추정하고 pose와 pose간의 관계를 edge로 표현함으로써 graph를 구성함
  - Back-end : Least Square Method를 사용해서 graph를 optimization함으로써 trajectory상에 존재하는 pose들을 update함

## 1. 원리 예제
- Front-end : Graph를 형성한다는 것은 information matrix와 information vector을 생성함
- Back-end : 방정식이 여러개가 있을 때 그것의 해를 구하는 과정을 거칢

#### - How to build pose graph 
- odometry, lidar 센서가 있다고 할 때
  - odometry를 통해 형성하는 방법
  - lidar를 통한 icp를 통해 형성하는 방법도 있음
    - 이전 pose를 기반으로 node를 만들고, 관측값으로 edge를 만들고 다음 pose값을 node로 형성 ... 해서 graph를 형성
    - 이렇듯 edge는 관측값으로 형성되며 즉, 센서값이며 이 edge에는 constraint가 존재하게됨
    - constraint : node와 node 사이의 확률분포에서 나온 마할라노비스 거리

##### - Example

![image](https://user-images.githubusercontent.com/108650199/193961650-b88fb1c2-0b10-4cb8-9da8-5fff14f71588.png)

![image](https://user-images.githubusercontent.com/108650199/193961595-7b2d12bb-1c83-499a-b154-0e6f5dfa9e1c.png)

  - xi, xj pose에 zi라는 관측값을 가진다면 이 셋의 확률 분포는 p(zi|xi,xj) 와 같이 매핑할 수 있음
    - p(zi|xi,xj) : xi와 xj라는 pose가 있을 때, zi가 관측될 확률
      - hi=(xi-1 º xj) 예상되는 transformation hi인데, 이런 transformation이 관측될 때 zi라는 관측이 얻어질 확률
      - 이 확률 분포에 마할라노비스 거리를 constraint로 가져오게 됨
      - 나중에는 이 constraint를 최소화 하는 쪽으로 optimization 하게 됨
      - 즉, p(zi|xi,xj)가 maximize 하게 됨 > 왜냐면 hi 확률 분포 계산에서 -가 앞에 붙기때문에 반대로

##### - Example Error function
![image](https://user-images.githubusercontent.com/108650199/193961688-ac033ff1-dc4b-4eb3-9612-a663a1210d76.png)
  
- 다시 정리하자면, Error function은 아래와 같음
  - Zij는 측정값으로 센서를 통해 xi에서 바라본 xj의 값을 의미하고, (X−1iXj)는 node에 저장된 값을 활용해서 xi에서 바라본 xj의 값을 의미
  - t2v는 Transformation to vector라는 함수를 의미
   
## 2. Graph Optimization
- constraint : 측정의 불확실성의 척도 (measure of uncertainty)
  - graph optimization은 이러한 uncertainty를 minimize 할 수 있는 pose들의 set을 구하는 것

  ![image](https://user-images.githubusercontent.com/108650199/193962191-7d484550-3781-49d9-aac8-f2a03177f432.png)

  - 이러한 constraint(![image](https://user-images.githubusercontent.com/108650199/193962393-229eb6aa-ba07-4cd9-b3a8-a6e4412ad366.png))의 합(x*)을 가장 작게할 수 있는 pose 집합을 구해내는 것

#### - Least Square Method in SLAM
- 실제 SLAM은 Overdetermined System (# of states < # of observations) : 즉, linear 하지 못함
  - [x,y,heading] = pose 로 하여 pose i, pose j node를 생성하고 zij (관측값) edge로 graph를 생성한다면 이 zij transformation은 🌟️nonlinear함🌟️
    - 따라서, 위처럼 X*를 바로 구할 수 없음

- 따라서 매 step마다 ![image](https://user-images.githubusercontent.com/108650199/193962890-c0c61bb9-43d6-4233-ab9b-aba9576f88fa.png)를 구함
  - 왜냐하면 non-linear least square을 풀 수 없기 때문
  - 즉, linearize (Gauss-Newton) 을 해서 ![image](https://user-images.githubusercontent.com/108650199/193963178-e24e3c07-515c-42e2-b549-84dd9ec6e47e.png) ![image](https://user-images.githubusercontent.com/108650199/193963195-96c4f44e-c758-455d-a20f-c67e824f3521.png) 이렇게 최적화하겠음


##### - Gauss-Newton 방법을 활용한 최적화

![image](https://user-images.githubusercontent.com/108650199/193963560-e3faa7bb-8e48-4e0b-b7b7-8d8493185aee.png)

- 1) Error function 정의
- 2) Error function 선형화(Linearize)
- 3) Jacobian을 활용하여 미분값 계산
- 4) 미분값 0으로 만드는 값 찾기
- 5) 선형 시스템 해 구하기
- 6) Iteration을 통해 최적의 값 찾기
