import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():
    # Launch Arguments
    gazebo_pkg_name = 'robot_gazebo_ros2_control'
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    robot_name_in_model = 'robot'
    urdf_file_path = 'urdf/robot_8leg.urdf'
    #world_file_path = 'worlds/basic.world'
    world_file_path = 'worlds/basic.world'
    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.3'
    spawn_yaw_val = '0.0'

    gazebo_pkg_share = FindPackageShare(package=gazebo_pkg_name).find(gazebo_pkg_name)
    world_path = os.path.join(gazebo_pkg_share, world_file_path)

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
                launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]
                #launch_arguments={'world': world_path}.items(),

             )

    #ignition_diffbot_description_path = os.path.join(
    #    get_package_share_directory('ignition_diffbot_description'))

    #xacro_file = os.path.join(ignition_diffbot_description_path,
    #                          'robots',
    #                          'diffbot.urdf.xacro')
    # xacroをロード
    #doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})
    # xacroを展開してURDFを生成
    #robot_desc = doc.toprettyxml(indent='  ')

    urdf_model_path = os.path.join(gazebo_pkg_share, urdf_file_path)

    doc = xacro.parse(open(urdf_model_path))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    #rviz_config_file = os.path.join(ignition_diffbot_description_path, 'config', 'diffbot_config.rviz')
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                '-entity', robot_name_in_model,
                '-x', spawn_x_val,
                '-y', spawn_y_val,
                '-z', spawn_z_val,
                '-Y', spawn_yaw_val,
                ],
    )

    gz_spawn_field = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
            #フィールドのsdfファイルを指定
        arguments=['-file', PathJoinSubstitution([
                        gazebo_pkg_share, 
                        "field", "step.sdf"]),
                   '-allow_renaming', 'false'
                '-x', '0.',
                '-y', '0.',
                '-z', '0.',
                ],
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',  '--set-state', 'active','joint_trajectory_controller'],
        output='screen'
    )

    # Bridge
    #bridge = Node(
    #    package='ros_gz_bridge',
    #    executable='parameter_bridge',
    #    arguments=['/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
    #               '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
    #               '/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
    #               '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
    #               '/depth_camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
    #               '/depth_camera/image_raw/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked'],
    #    output='screen'
    #)
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (IGN -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint states (IGN -> ROS2)
            '/world/empty/model/robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/wrench@geometry_msgs/msg/Wrench[gz.msgs.Wrench',
            '/contacts@ros_gz_interfaces/msg/Contact[gz.msgs.Contact',
            # odometry (ROS2 -> IGN)
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        ],
        remappings=[
            ('/world/empty/model/robot/joint_state', 'gz_joint_states'),
        ],
        output='screen'
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=['-d', 'check.rviz'],
    )
    
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
               target_action=load_joint_state_controller,
               on_exit=[load_joint_trajectory_controller],
            )
        ),
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        gz_spawn_field,
        bridge,
        #velocity_converter,
        rviz,
    ])

