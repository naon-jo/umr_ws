# Autonomous mobile robot

## 1. Overview

### Purpose

1. 실내 공간에서 주어진 작업(task)을 수행하는 자율주행 로봇을 Turtlebot3에 구현한다.
2. 거리/방향센서(LiDAR)와 비전센서(RBG Camera)를 이용하여 물체를 인지하고 로봇 행동을 계획하는 알고리즘을 구현한다.
3. 자율주행을 위한 센서 데이터 수집 및 처리, 상태 추정, 경로생성, 제어 기술을 학습하고 구현한다. (Future work)

### Methods
1. ROS2
2. Python
3. Matlab/Simulink
4. Math(AI)

<br>

## 2. System Architecture
### ROS2 Interface

<a href="https://naon-jo.github.io/posts/umr-ws-System-Architecture/">
    <img src="https://github.com/user-attachments/assets/a11b244e-20ca-4f2c-8210-0924f39ae14a" alt="ros2_interface" width="900"/>
</a>

<br>

- Task Manager : task를 의미있는 실행 단위인 phase로 해석
- Goal Executor : phase를 실제 수행 가능한 명령으로 변환하여 실행

<br>

## 2. Front Obstacle Detection
### LiDAR Obstacle Detection

라이다의 정보는 고정 각도마다 상대거리값을 출력시켜주는 포인트 클라우드 형식이다.

```check_obstacle_front``` 함수는 라이다 센서 데이터를 방위각과 거리로 필터링하여 전방 장애물을 감지하고, 감지된 장애물까지의 거리를 반환한다.

<br>

## 3. Task Definition
## Task

로봇이 수행해야 하는 하나의 작업(task)는 일련의 세부 행동들(phase)로 구성된다. 예를 들어, 
- ```delivery``` task는 여러 개의 ```pick_up```과 ```drop_off``` phase를 포함할 수 있다.
- ```go_to_places``` task는 하나 이상의 ```go_to_place``` phase로 정의된다.

### go_to_places

go_to_places는 로봇이 여러 장소를 순차적으로 방문하는 일련의 동작을 정의한 task이다.

## Phase

로봇이 실제 실행 가능한 행동 단위이다. phase들이 시퀀스로 구성되어 하나의 phases를 이룰 수 있다.

### go_to_place

목적지 근처에서 정지하여 목적지에 사람 또는 장애물의 존재 여부를 확인하고, 목적지로 접근하는 행동으로 정의된다.

<a href="https://naon-jo.github.io/posts/umr-ws-go-to-place/">
    <img src="https://github.com/user-attachments/assets/7af68ea8-a8a8-46f4-b50c-240d13831cb9" alt="Image" width="300"/>
</a>

<br>

## 4. Simulation

<img src="https://github.com/user-attachments/assets/fd0a02e0-b779-44b7-8f53-28361cbb95e3" alt="Image" width="350"/>

<img src="https://github.com/user-attachments/assets/5e2044a2-c985-4751-905b-924962f9dc23" alt="Image" width="350"/>

<br>
