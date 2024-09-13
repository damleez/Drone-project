# Jeton Orin AGX developer kit

![image](https://github.com/damleez/Drone-project/assets/108650199/40d7b99c-3a57-49d4-9543-0d224081267c)

![image](https://github.com/damleez/Drone-project/assets/108650199/321395bb-e7b2-4aae-bf07-e9403ec900ad)

## 1. Instruction
### 1.1 Host PC에 L4T 및 JatPack 설치

![image](https://github.com/damleez/Drone-project/assets/108650199/935e8cb4-6a97-4c27-a2b9-3023013c1543)

- [NVIDIA 홈페이지](https://developer.nvidia.com/embedded/jetson-linux) 에서 L4T Driver Package[BSP] , Sample Root Filesystem 다운로드
- NVIDIA 홈페이지에서 SDK Manager 다운로드(회원가입 필요)

#### Error

![image](https://github.com/damleez/Drone-project/assets/108650199/962a3f35-43d8-4da6-ba27-500bd3463ea4)

### 1.2 Host PC에 SDK manger 설치
- [NVIDA 홈페이지](https://developer.nvidia.com/sdk-manager) 에서 .deb파일 다운로드 후 실행

```
sudo apt install ./sdkmanager_*-*_amd64.deb 
sdkmanager
```

![image](https://github.com/damleez/Drone-project/assets/108650199/26d548d3-8c3d-4152-ad45-749a5618a63a)

![image](https://github.com/damleez/Drone-project/assets/108650199/6edc3f5e-f9ce-477f-a0a2-c0612cf713b1)

#### Manual Setup : 처음 설치하거나 기존에 설치되어 있는 경우
1. Manual setup
2. 파워어답터를 연결하지만, 전원은 키지 않음
3. USB C-Type connector를 jetson과 호스트PC에 연결
(USB C타입은 Power LED 옆에 있는 단자를 사용, 파워어답터 위의 단자는 연결이 안됨)
4. 중간(Force Recovery) 버튼을 누르고 떼지않음
5. 왼쪽(power) 버튼을 누르고 떼지않음
6. 두 버튼을 뗀다

# Jetson Orin NX 16GB
## 1. Install
- [nvidia 홈페이지](https://www.waveshare.com/wiki/JETSON-ORIN-NX-16G-DEV-KIT)를 참조하여 install
- 점퍼캡
```
1. Choose whether to put your Jetson Orin NX 16GB into Force Recovery Mode via Manual Setup or Automatic Setup. Choose Automatic Setup only if the device has already been flashed and is currently running.
2. Ensure the device is powered off and the power adapter disconnected.
3. Verify the storage device for flashing is connected to the Jetson Orin NX 16GB module.
4. Place a jumper across the Force Recovery Mode pins.
  These are pins 9 and 10 of the Button Header [J14].
5. Connect your host computer to the device's USB Type-C connector.
6. Connect the power adapter to the Power Jack [J16].
  The device will automatically power on in Force Recovery Mode.
7. Remove the jumper from the Force Recovery Mode pins.
```

![image](https://github.com/damleez/Drone-project/assets/108650199/01635a54-ed29-42ac-a84d-88da54004617)

## 2. setting
- ssd이므로 nvme이며, 인터넷도 연결해놔야함
- ip주소 ifconfig 확인 하니 ```192.168.0.71``` 임
- 계속해서 점퍼캡은 연결해놓기
