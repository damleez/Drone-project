## 220816

1. Home/ardupilot/Tools/autotest/default_params
- copter.parm의 18line FRAME_CLASS를 1에서 2로 변경

2. gazebo는 내꺼 그냥 틀고 simvehicle 어쩌고를 수정해서 말 듣게 하면 안되나

![image](https://user-images.githubusercontent.com/108650199/184817115-57d4696b-2100-40b1-9ac3-ed0ab97ab62a.png)
  
![image](https://user-images.githubusercontent.com/108650199/184817960-e2ea742a-a16f-4050-9204-4263870cb799.png)

- udp, tcp등이 안맞음 gazebo master는 tcp:127.0.0.1:5760, 내꺼 drone model launch로 키면 master가 local host
  - 잘 모르는 분야라서 찾아봐야됨
- ../Tools/autotest/sim_vehicle.py > map, console 다 뺌
- 이후 mode GUIDED, arm throttle, takeoff 2 시 take off됨
  - 근데 이거부터 알려면 3번 해결해야함 + 날개 안붙어있음

3. https://github.com/ethz-asl/rotors_simulator/blob/master/rotors_gazebo/models/typhoon_h480/typhoon_h480.sdf
- 여기 보면 지금 rotor에 motor,, 이거 나중에 추가해야될듯

===
지금 1 2 3 해결
===

## 220817

1. motor lib확인 후 제대로 넣었음
2. 날개랑 몸통이 안붙어있는게 collision 때문일수도 있나? joint
3. TF문제인듯? TF가 이상한데 있음

![default_gzclient_camera(1)-2022-08-17T15_24_10 138852](https://user-images.githubusercontent.com/108650199/185049326-c2ca9032-c9ca-437f-8d67-c7310f38abe7.jpg)

  - 걍 고정되어있는듯 스케일 변화하면서 해봤는데 원래 그림에 맞출수록 TF도 다시 줄어듦 왜지감자
  - xacro는 tf필요없음.. joint state publish가 쉽게 변환해주기 때문 그면 어케 해야하나요

4. 오류
```
Topic [//home/dam/catkin_ws/src/dam-drone/xacro/drone_base.xacro/joint_cmd] is not valid.
Service [//home/dam/catkin_ws/src/dam-drone/xacro/drone_base.xacro/joint_cmd_req] is not valid.
```

5. libLiftDragPlugin.so 을 이용해보기 모터 어쩌고 말고


---

민재 선배 찬스
1. 날개 고정
- urdf는 sdf와 다르게 link맨 위에서 pose하면 다 따라오는게 아님 독립적임
- 모든것이 원점기준으로 이미지파일이 (dae, stl)이 모델링 되어있는데, 날개는 원점기준으로 떨어진 곳에 날개모양을 만들어났기 때문에
- 비쥬얼 pose설정을해서 원점으로 들고옴. collision은 원점에서 봉 설정되어있기 때문에...
- 그래서 축이 나는 전에 두배이상으로 되어있었던거임 scale 때문
- 나중에 정리하자
