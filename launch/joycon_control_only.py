from launch import LaunchDescription
from launch.actions import Shutdown
from ament_index_python.packages import get_package_prefix
import launch_ros.actions
package = "minirys_drivers"

def generate_launch_description():
	path = get_package_prefix(package)
	path_list = path.split('/')
	del path_list[-2]
	path_list.insert(-1, 'src')
	full_path = '/'.join(path_list)

	return LaunchDescription([
		launch_ros.actions.Node(
			# the name of the executable is set in CMakeLists.txt, towards the end of
			# the file, in add_executable(...) and the directives following it
			package= package,
			executable='imu_sensor_node',
			output='screen',
			parameters=[full_path + "/yaml/main_params.yaml"],
			on_exit=Shutdown(),
		),
		launch_ros.actions.Node(
			# the name of the executable is set in CMakeLists.txt, towards the end of
			# the file, in add_executable(...) and the directives following it
			package= package,
			executable='communication_node',
			output='screen',
			parameters=[full_path + "/yaml/main_params.yaml"],
			on_exit=Shutdown(),
		),
		launch_ros.actions.Node(
			# the name of the executable is set in CMakeLists.txt, towards the end of
			# the file, in add_executable(...) and the directives following it
			package= package,
			executable='motors_controller_node',
			output='screen',
			parameters=[full_path + "/yaml/main_params.yaml"],
			on_exit=Shutdown(),
		),
		launch_ros.actions.Node(
			# the name of the executable is set in CMakeLists.txt, towards the end of
			# the file, in add_executable(...) and the directives following it
			package= package,
			executable='joycon_receiver_node',
			output='screen',
			parameters=[full_path + "/yaml/main_params.yaml"],
			on_exit=Shutdown(),
		),
	])
