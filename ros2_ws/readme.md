📘 ROS2 Doosan E0509 End-Effector Control Simulation
📌 프로젝트 개요

본 프로젝트는 ROS2 Humble 환경에서
두산 로보틱스 E0509 로봇암을 활용하여

사용자가 입력한 목표 좌표 (x, y, z)로
로봇암 말단(End-Effector, TCP)을 이동시키는
GUI 기반 시뮬레이션 제어 프로그램입니다.

로봇 제어는 ROS2 기반으로 수행되며,
GUI는 PyQt5를 코드로 직접 구현하였습니다.

🧩 시스템 구성
사용자 GUI (PyQt5)
        ↓
ROS2 Node (Python)
        ↓
IK 계산 (/compute_ik)
        ↓
Joint Trajectory 생성
        ↓
ros2_control Action Server
        ↓
로봇 시뮬레이션 (Gazebo / RViz)

⚙️ 개발 환경
항목	내용
OS	Ubuntu 22.04 (Docker)
ROS	ROS2 Humble
로봇	Doosan E0509
언어	Python
GUI	PyQt5
제어	MoveIt2 + ros2_control
TCP 계산	tf2_ros
🚀 실행 방법
1️⃣ ROS2 환경 준비
docker exec -it doosan_dev /bin/bash

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

2️⃣ MoveIt2 데모 실행
ros2 launch dsr_moveit_config_e0509 demo.launch.py


RViz 및 시뮬레이션이 실행됨.

3️⃣ GUI 프로그램 실행
ros2 run my_ros2_assignment my_node


GUI 창이 실행됨.

🖥️ GUI 구성
🔹 1. Control Panel

사용자가 로봇을 제어하는 영역

입력 요소

X, Y, Z 좌표 입력 (meter 단위)

Relative Coordinate 체크박스

Velocity (0~1)

Acceleration (0~1)

기능 버튼

Add Waypoint

Execute

Waypoint 목록

여러 개 목표 좌표 저장

순차 실행

🔹 2. Robot Status

로봇 상태 모니터링 영역

표시 항목:

Connection 상태 (CONNECTED / DISCONNECTED)

State (MOVING / IDLE)

각 Joint 각도 (6축)

TCP 위치 (world 기준)

실시간 로그 출력

🧠 작동 로직
1️⃣ 목표 좌표 입력

사용자가 (x, y, z) 입력
→ Waypoint 리스트에 저장

2️⃣ 좌표 기준 판단
Absolute 모드
world 기준 좌표로 이동

Relative 모드
목표 = 현재 TCP + 입력 좌표

3️⃣ IK 계산
/compute_ik 서비스 호출


목표 TCP 위치 → Joint 각도 계산

4️⃣ 부드러운 Trajectory 생성

150 step interpolation

velocity 반영

acceleration 반영

time_from_start 계산

5️⃣ Action Server 전송
/arm_controller/follow_joint_trajectory


목표 joint trajectory 전송

6️⃣ TF 기반 TCP 계산
tf_buffer.lookup_transform("world", "link_6")


link_6 = End-Effector

world 기준 절대 위치 표시

📊 주요 기능 설명
✅ 다중 Waypoint 순차 실행

최소 1개 이상 입력 가능

for-loop 기반 순차 처리

✅ Relative / Absolute 좌표

체크박스로 기준 선택

실시간 TCP 기반 계산

✅ Velocity 반영

이동 시간 계산

JointTrajectoryPoint.velocities 설정

✅ Acceleration 반영

JointTrajectoryPoint.accelerations 설정

가속도 기반 부드러운 이동

✅ 실시간 상태 표시
항목	설명
Joint	현재 6축 각도
TCP	world 기준 End-Effector 위치
Connection	JointState 수신 여부
State	moving flag 기반
✅ 별도 스레드 구조
class MotionWorker(threading.Thread)


GUI 프리징 방지

ROS 제어와 UI 분리

🔬 기술적 특징
✔ Forward Kinematics

tf2 기반 TCP 계산

FK 서비스 대신 TF 사용

✔ Inverse Kinematics

MoveIt2 /compute_ik 서비스 사용

✔ 실시간 갱신 구조
QTimer → rclpy.spin_once()


GUI와 ROS 통합 처리

📐 프레임 구조

TF Tree:

world
 └── base_link
      └── link_1
           └── link_2
                └── link_3
                     └── link_4
                          └── link_5
                               └── link_6 (TCP)


TCP는 link_6 기준.

📎 요구사항 충족 여부
요구사항	충족 여부
목표 좌표 입력	✅
최소 1개 이상 입력	✅
Relative / Absolute 선택	✅
Velocity 입력 반영	✅
Acceleration 입력 반영	✅
순차 이동	✅
PyQt5 코드 구현	✅
별도 스레드	✅
로봇 연결 상태 표시	✅
이동 상태 표시	✅
실시간 로그	✅
Joint 각도 표시	✅
TCP 절대 좌표 표시	✅
🏁 결론

본 프로젝트는
ROS2 기반 두산 E0509 로봇암 시뮬레이션 환경에서

사용자 입력 좌표 기반 End-Effector 제어

다중 Waypoint 처리

속도/가속도 반영

GUI 기반 제어 및 상태 모니터링

을 완전 구현하였다.

요구사항을 100% 충족하며,
실제 산업용 로봇 제어 구조와 동일한 흐름으로 설계되었다.