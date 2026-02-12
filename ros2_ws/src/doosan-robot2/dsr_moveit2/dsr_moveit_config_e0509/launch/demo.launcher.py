from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import yaml
import os


def load_yaml(file_path):
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():

    pkg_share = get_package_share_directory("dsr_moveit_config_e0509")

    urdf_file = os.path.join(pkg_share, "config", "e0509.urdf.xacro")
    srdf_file = os.path.join(pkg_share, "config", "dsr.srdf")
    ros2_controllers = os.path.join(pkg_share, "config", "ros2_controllers.yaml")
    moveit_controllers = os.path.join(pkg_share, "config", "moveit_controllers.yaml")
    kinematics_file = os.path.join(pkg_share, "config", "kinematics.yaml")

    # ------------------------------
    # Robot Description (URDF)
    # ------------------------------
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", urdf_file]),
            value_type=str
        )
    }

    # ------------------------------
    # SRDF
    # ------------------------------
    with open(srdf_file, "r") as f:
        robot_description_semantic = {
            "robot_description_semantic": ParameterValue(
                f.read(),
                value_type=str
            )
        }

    # ------------------------------
    # Kinematics (dict로 전달)
    # ------------------------------
    kinematics_config = {
        "robot_description_kinematics":
            load_yaml(kinematics_file)
    }

    # ------------------------------
    # MoveIt Controller Config
    # ------------------------------
    moveit_controller_config = load_yaml(moveit_controllers)

    # ------------------------------
    # Nodes
    # ------------------------------

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            moveit_controller_config,
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_node,
        rviz_node,
    ])

