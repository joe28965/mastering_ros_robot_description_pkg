import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
	main_dir = get_package_share_directory('mastering_ros_robot_description_pkg')

	rviz_config_file = LaunchConfiguration('rviz_config')

	declare_rviz_config_file_cmd = DeclareLaunchArgument(
		'rviz_config',
		default_value=os.path.join(main_dir, 'rviz', 'urdf.rviz'),
		description='Full path to the RVIZ config file to use')

	# Specify the actions
	urdf = os.path.join(main_dir, 'urdf', 'pan_tilt.urdf')

	start_gazebo_server_cmd = ExecuteProcess(
		cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
		cwd=[main_dir], output='screen')

	start_gazebo_client_cmd = ExecuteProcess(
		cmd=['gzclient'],
		cwd=[main_dir], output='screen')

	start_gazebo_ros_spawner_cmd = 	Node(
		package='gazebo_ros', 
		executable='spawn_entity.py',
		arguments=['-entity', 'pan_tilt',
								'-file', urdf,
								'-z', '0.3',
							],
		output='screen')

	start_robot_state_publisher_cmd = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher',
		output='screen',
		arguments=[urdf])

	start_rviz_cmd = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		arguments=['-d', rviz_config_file],
		output='screen')

	exit_event_handler = RegisterEventHandler(
		event_handler=OnProcessExit(
			target_action=start_rviz_cmd,
			on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))

	# Create the launch description and populate
	ld = LaunchDescription()

	ld.add_action(declare_rviz_config_file_cmd)

	ld.add_action(start_gazebo_server_cmd)
	ld.add_action(start_gazebo_client_cmd)
	ld.add_action(start_gazebo_ros_spawner_cmd)

	ld.add_action(start_robot_state_publisher_cmd)
	ld.add_action(start_rviz_cmd)
	ld.add_action(exit_event_handler)

	return ld