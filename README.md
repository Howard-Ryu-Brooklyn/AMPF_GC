# AMPF — Ground Control (ROS2)

[AMPF_MATLAB](https://github.com/Howard-Ryu-Brooklyn/AMPF_MATLAB)에서 다룬 편대 제어 알고리즘을
실제 로봇 위에서 돌리기 위한 ROS2 구현입니다. 시뮬레이션에서 성립한 제어기가 실물에서는 어떻게
달라지는지가 여기서부터 드러납니다.

## 구성

- **`formation_controller`** — follower1/2 편대 제어기. MATLAB에서 검증한 제어 법칙을
  실시간 ROS2 노드로 이식.
- **`formation_experiment`** — ArUco 마커·ZED 카메라 기반 상대 위치 추정, 실험용 launch 구성.
- **`sensing`** — YOLOv8 기반 인식, ZED 카메라 연동 모듈.

## 실기에서 새로 다뤄야 했던 것

시뮬레이션은 이상적인 위치·거리 측정을 가정하지만, 실물에서는 카메라 인식 지연, 통신 지연,
좌표계 변환이 그대로 오차로 들어옵니다. 이 저장소는 그 격차를 메우는 센싱·통신 레이어이고,
편대 제어 알고리즘 자체는 [AMPF_MATLAB](https://github.com/Howard-Ryu-Brooklyn/AMPF_MATLAB)에서
이어집니다.

## 관련 저장소

| 저장소 | 역할 |
|---|---|
| [AMPF_MATLAB](https://github.com/Howard-Ryu-Brooklyn/AMPF_MATLAB) | 알고리즘 이론·시뮬레이션·학위논문 |
| AMPF_GC (이 저장소) | 지상 관제 PC — ROS2 편대 제어기, 비전 센싱 |
| [AMPF_Jetson](https://github.com/Howard-Ryu-Brooklyn/AMPF_Jetson) | 로봇 온보드(Jetson Nano) — 구동, LiDAR/UWB |

## 실행 환경

Ubuntu 22.04, ROS2 Humble. ZED SDK, crazyswarm2, webots-ros2 설치가 필요합니다 (패키지별
설치 순서는 각 하위 폴더 참고).

------------------

Since ZED-ros2-wrapper and crazyswarm2 requires Ubuntu 22.04,
Ubuntu 22.04 jammy is recommended.

Installation Order
------------------
- - -
1. zed sdk    
<https://www.stereolabs.com/docs/installation/linux/>

cuda will be automatically installed while zed sdk installed.   
In order to check your zed sdk installed correctly, going through following step is strongly recommended   
<https://www.stereolabs.com/docs/app-development/cpp/linux/>   

2. ROS2 Humble    
<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>

3. zed-ros2-wrapper & zed-ros2-examples     
<https://github.com/stereolabs/zed-ros2-wrapper>    
<https://github.com/stereolabs/zed-ros2-examples>

5. crazyswarm2    
<https://github.com/IMRCLab/crazyswarm2>

6. webot and webot-ros2
sudo apt-get install ros-humble-webot-ros2*
when you run the launch file, webot will be automatically installed.

7. ultralytics
   pip install ultralytics

우분투컴 
ID: humble
PW: 123123

위봇 시뮬레이션 실행순서
- Terminal1
  webot_sim
- Terminal2
  controller

하드웨어 실행순서
- Terminal1
   sshf1 -> bringup
- Terminal2
   sshf2 -> bringup
- Terminal3
   sensing
- Terminal4
  controller

