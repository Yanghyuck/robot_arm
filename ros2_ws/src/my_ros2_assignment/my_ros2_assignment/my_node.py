import sys
import threading
import numpy as np

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit,
    QPushButton, QVBoxLayout, QTextEdit,
    QCheckBox, QGroupBox, QListWidget
)
from PyQt5.QtCore import QTimer, QObject, pyqtSignal, Qt

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.duration import Duration


# =========================================================
# Logger
# =========================================================

class QtLogger(QObject):
    log_signal = pyqtSignal(str)


# =========================================================
# Motion Worker Thread
# =========================================================

class MotionWorker(threading.Thread):
    def __init__(self, node, waypoints, vel, acc, relative):
        super().__init__()
        self.node = node
        self.waypoints = waypoints
        self.vel = vel
        self.acc = acc
        self.relative = relative

    def run(self):
        self.node.execute_waypoints(
            self.waypoints,
            self.vel,
            self.acc,
            self.relative
        )


# =========================================================
# ROS Node
# =========================================================

class RobotNode(Node):

    def __init__(self, logger):
        super().__init__("e0509_gui_node")

        self.logger = logger
        self.current_joints = [0.0]*6
        self.connected = False
        self.moving = False

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # IK
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        # Trajectory
        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory"
        )

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10
        )

    # ------------------------------------------------

    def log(self, msg):
        print(msg)
        self.logger.log_signal.emit(msg)

    # ------------------------------------------------

    def joint_callback(self, msg):
        if len(msg.position) >= 6:
            self.current_joints = list(msg.position[:6])
            self.connected = True

    # ------------------------------------------------
    # TCP 실시간 표시 (TF 기반)
    # ------------------------------------------------

    def get_tcp_pose(self):

        try:
            trans = self.tf_buffer.lookup_transform(
                "world",
                "link_6",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )

            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z

            return (round(x,3), round(y,3), round(z,3))

        except TransformException:
            return None

    # ------------------------------------------------
    # 부드러운 Trajectory 생성 (Acceleration 반영)
    # ------------------------------------------------

    def generate_smooth_trajectory(self, start, goal, vel_scale, acc_scale):

        start = np.array(start)
        goal = np.array(goal)

        steps = 150

        duration = max(2.0 / max(vel_scale,0.01), 1.0)

        traj = JointTrajectory()
        traj.joint_names = [
            "joint_1","joint_2","joint_3",
            "joint_4","joint_5","joint_6"
        ]

        for i in range(steps+1):

            ratio = i/steps
            interp = start + (goal-start)*ratio

            point = JointTrajectoryPoint()
            point.positions = interp.tolist()

            t = duration * ratio
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t%1)*1e9)

            # velocity & acceleration 반영
            velocity = (goal-start)/duration
            point.velocities = (velocity*vel_scale).tolist()
            point.accelerations = (velocity*acc_scale).tolist()

            traj.points.append(point)

        return traj

    # ------------------------------------------------
    # Waypoint 실행
    # ------------------------------------------------

    def execute_waypoints(self, waypoints, vel, acc, relative):

        if len(waypoints) == 0:
            self.log("[ERROR] No Waypoints")
            return

        self.moving = True
        self.log("[INFO] Start Motion")

        for wp in waypoints:

            x,y,z = wp

            if relative:
                tcp = self.get_tcp_pose()
                if tcp:
                    x += tcp[0]
                    y += tcp[1]
                    z += tcp[2]

            self.log(f"[INFO] Moving to ({x:.3f},{y:.3f},{z:.3f})")

            req = GetPositionIK.Request()
            req.ik_request.group_name = "manipulator"

            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0

            req.ik_request.pose_stamped = pose

            future = self.ik_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            res = future.result()
            if not res or res.error_code.val != 1:
                self.log("[ERROR] IK Failed")
                continue

            goal_joints = res.solution.joint_state.position[:6]

            traj = self.generate_smooth_trajectory(
                self.current_joints,
                goal_joints,
                vel,
                acc
            )

            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj

            send_future = self.traj_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)

        self.log("[SUCCESS] All Waypoints Done")
        self.moving = False


# =========================================================
# GUI
# =========================================================

class RobotGUI(QWidget):

    def __init__(self, node, logger):
        super().__init__()
        self.node = node
        self.waypoints = []

        self.setWindowTitle("E0509 Assignment GUI")
        self.resize(850,800)

        self.init_ui()

        logger.log_signal.connect(self.append_log, Qt.QueuedConnection)

        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(50)

    # ------------------------------------------------

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        self.update_status()

    # ------------------------------------------------

    def init_ui(self):

        main_layout = QVBoxLayout()

        control_group = QGroupBox("Control Panel")
        control_layout = QVBoxLayout()

        self.x_input = QLineEdit("0.3")
        self.y_input = QLineEdit("0.0")
        self.z_input = QLineEdit("0.3")

        self.rel_check = QCheckBox("Relative Coordinate")

        self.vel_input = QLineEdit("0.3")
        self.acc_input = QLineEdit("0.3")

        self.wp_list = QListWidget()

        add_btn = QPushButton("Add Waypoint")
        exec_btn = QPushButton("Execute")

        add_btn.clicked.connect(self.add_waypoint)
        exec_btn.clicked.connect(self.start_motion)

        control_layout.addWidget(QLabel("X"))
        control_layout.addWidget(self.x_input)
        control_layout.addWidget(QLabel("Y"))
        control_layout.addWidget(self.y_input)
        control_layout.addWidget(QLabel("Z"))
        control_layout.addWidget(self.z_input)
        control_layout.addWidget(self.rel_check)
        control_layout.addWidget(QLabel("Velocity (0~1)"))
        control_layout.addWidget(self.vel_input)
        control_layout.addWidget(QLabel("Acceleration (0~1)"))
        control_layout.addWidget(self.acc_input)
        control_layout.addWidget(self.wp_list)
        control_layout.addWidget(add_btn)
        control_layout.addWidget(exec_btn)

        control_group.setLayout(control_layout)

        status_group = QGroupBox("Robot Status")
        status_layout = QVBoxLayout()

        self.conn_label = QLabel("Connection: DISCONNECTED")
        self.state_label = QLabel("State: IDLE")
        self.joint_label = QLabel("Joint: 0,0,0,0,0,0")
        self.tcp_label = QLabel("TCP: WAITING TF")

        self.log_area = QTextEdit()
        self.log_area.setReadOnly(True)

        status_layout.addWidget(self.conn_label)
        status_layout.addWidget(self.state_label)
        status_layout.addWidget(self.joint_label)
        status_layout.addWidget(self.tcp_label)
        status_layout.addWidget(QLabel("Realtime Log"))
        status_layout.addWidget(self.log_area)

        status_group.setLayout(status_layout)

        main_layout.addWidget(control_group)
        main_layout.addWidget(status_group)

        self.setLayout(main_layout)

    # ------------------------------------------------

    def add_waypoint(self):
        x = float(self.x_input.text())
        y = float(self.y_input.text())
        z = float(self.z_input.text())
        self.waypoints.append((x,y,z))
        self.wp_list.addItem(f"({x},{y},{z})")

    # ------------------------------------------------

    def start_motion(self):
        vel = float(self.vel_input.text())
        acc = float(self.acc_input.text())
        relative = self.rel_check.isChecked()

        worker = MotionWorker(
            self.node,
            self.waypoints.copy(),
            vel,
            acc,
            relative
        )
        worker.start()

    # ------------------------------------------------

    def append_log(self, msg):
        self.log_area.append(msg)

    # ------------------------------------------------

    def update_status(self):

        self.conn_label.setText(
            "Connection: CONNECTED" if self.node.connected else "Connection: DISCONNECTED"
        )

        self.state_label.setText(
            "State: MOVING" if self.node.moving else "State: IDLE"
        )

        joints = ", ".join(f"{j:.2f}" for j in self.node.current_joints)
        self.joint_label.setText(f"Joint: {joints}")

        tcp = self.node.get_tcp_pose()
        if tcp:
            self.tcp_label.setText(f"TCP: {tcp}")
        else:
            self.tcp_label.setText("TCP: WAITING TF")


# =========================================================
# MAIN
# =========================================================

def main():
    rclpy.init()

    logger = QtLogger()
    node = RobotNode(logger)

    app = QApplication(sys.argv)
    gui = RobotGUI(node, logger)
    gui.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

