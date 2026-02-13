#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('robot_ip', default_value=''),
        DeclareLaunchArgument('report_type', default_value='normal'),
        DeclareLaunchArgument('hw_ns', default_value='ufactory'),
        DeclareLaunchArgument('add_gripper', default_value='false'),
        DeclareLaunchArgument('start_controller', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_driver', default_value='false'),
        DeclareLaunchArgument('start_description', default_value='false'),
        DeclareLaunchArgument('start_realsense', default_value='true'),
        DeclareLaunchArgument('realsense_log_level', default_value='error'),
        DeclareLaunchArgument('marker_length', default_value='0.075'),
        DeclareLaunchArgument('image_topic', default_value='/camera/camera/color/image_rect_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera/color/camera_info'),
        DeclareLaunchArgument('target_marker_id', default_value='12'),
        DeclareLaunchArgument('marker_topic', default_value='/marker_poses'),
        DeclareLaunchArgument('min_samples', default_value='15'),
        DeclareLaunchArgument('base_frame', default_value='link_base'),
        DeclareLaunchArgument('ee_frame', default_value='link_eef'),
        DeclareLaunchArgument('camera_frame', default_value='camera_color_optical_frame'),
        DeclareLaunchArgument('result_child_frame', default_value='camera_color_optical_frame_calibrated'),
    ]

    robot_ip = LaunchConfiguration('robot_ip')
    report_type = LaunchConfiguration('report_type')
    hw_ns = LaunchConfiguration('hw_ns')
    add_gripper = LaunchConfiguration('add_gripper')
    start_controller = LaunchConfiguration('start_controller')
    start_rviz = LaunchConfiguration('start_rviz')
    start_driver = LaunchConfiguration('start_driver')
    start_description = LaunchConfiguration('start_description')
    start_realsense = LaunchConfiguration('start_realsense')
    realsense_log_level = LaunchConfiguration('realsense_log_level')

    marker_length = LaunchConfiguration('marker_length')
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    target_marker_id = LaunchConfiguration('target_marker_id')
    marker_topic = LaunchConfiguration('marker_topic')
    min_samples = LaunchConfiguration('min_samples')
    base_frame = LaunchConfiguration('base_frame')
    ee_frame = LaunchConfiguration('ee_frame')
    camera_frame = LaunchConfiguration('camera_frame')
    result_child_frame = LaunchConfiguration('result_child_frame')

    xarm_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('xarm_controller'), 'launch', '_robot_ros2_control.launch.py'])
        ),
        launch_arguments={
            'robot_ip': robot_ip,
            'report_type': report_type,
            'hw_ns': hw_ns,
            'add_gripper': add_gripper,
            'dof': '6',
            'robot_type': 'lite',
        }.items(),
        condition=IfCondition(start_controller),
    )

    xarm_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('xarm_api'), 'launch', 'lite6_driver.launch.py'])
        ),
        launch_arguments={
            'robot_ip': robot_ip,
            'report_type': report_type,
            'hw_ns': hw_ns,
        }.items(),
        condition=IfCondition(start_driver),
    )

    xarm_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('xarm_description'), 'launch', '_robot_joint_state.launch.py'])
        ),
        launch_arguments={
            'hw_ns': hw_ns,
            'dof': '6',
            'robot_type': 'lite',
        }.items(),
        condition=IfCondition(start_description),
    )

    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])
        ),
        launch_arguments={
            'depth_module.depth_profile': '1280x720x30',
            'pointcloud.enable': 'true',
            'log_level': realsense_log_level,
            'output': 'log',
        }.items(),
        condition=IfCondition(start_realsense),
    )

    calibration_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='calibration_rviz',
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution([FindPackageShare('hand_eye_calibration'), 'Config', 'Calibration_visual.rviz']),
        ],
        condition=IfCondition(start_rviz),
    )

    aruco_pose_node = Node(
        package='hand_eye_calibration',
        executable='aruco_pose_estimation',
        name='aruco_pose_estimation',
        output='log',
        arguments=['--ros-args', '--log-level', 'error'],
        parameters=[
            {
                'marker_length': marker_length,
                'image_topic': image_topic,
                'camera_info_topic': camera_info_topic,
                'target_marker_id': target_marker_id,
            }
        ],
    )

    hand_eye_calibrator_node = Node(
        package='hand_eye_calibration',
        executable='hand_eye_calibrator',
        name='hand_eye_calibrator',
        output='screen',
        parameters=[
            {
                'marker_topic': marker_topic,
                'min_samples': min_samples,
                'base_frame': base_frame,
                'ee_frame': ee_frame,
                'camera_frame': camera_frame,
                'result_child_frame': result_child_frame,
            }
        ],
    )

    return LaunchDescription(
        args
        + [
            xarm_controller,
            xarm_driver,
            xarm_description,
            realsense_camera,
            calibration_rviz,
            aruco_pose_node,
            hand_eye_calibrator_node,
        ]
    )
