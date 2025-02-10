from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare the use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time'
    )
    
    # Find the launch file of slam_toolbox
    slam_launch_file = FindPackageShare('slam_toolbox').find('slam_toolbox')
    slam_launch_path = slam_launch_file + '/launch/online_async_launch.py'
    
    # Include the SLAM toolbox launch file
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        slam_launch
    ])
