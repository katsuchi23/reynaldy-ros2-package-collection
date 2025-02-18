import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

''' 
Note:
- When the launch file gets launched, the file gets launched from the ros2_ws/install/share folder instead of the ros2_ws/src folder. Hence the os.getcwd() will also return /home/delta/ros2_ws/

'''

def generate_launch_description():
    # Paths to dependencies
    nav2_bringup_dir = get_package_share_directory('my_nav2_bringup') # this is directory in install not in src so different 
    nav2_collision_monitor_dir = get_package_share_directory('nav2_collision_monitor')
    # os.getcwd() will return /ros2_ws and change depends on the directory of the terminal not where the file is located/runned
    # current_package_dir = os.path.join(os.getcwd(), 'src', 'my_nav2_brinup') # this is to get package directory INSIDE SRC not inside install folder (see note because when the file is runned, it doesn't runned from the src file
    current_dir = os.path.dirname(os.path.realpath(__file__)) # this is to get package directory INSIDE SRC not inside install folder (see note because when the file is runned, it doesn't runned from the src file)
    current_package_dir = os.path.abspath(os.path.join(current_dir, '..', '..', '..', '..', '..')) # return /ros2_ws/install/my_nav2_bringup/share/my_nav2_bringup no matter what terminal the file is running. this is because of the command os.dirname(os.path.realpath(__file__)) which is the path where the file is runned
    # print(x)
    # we can also just define the path manually which is what we will do here
    # current_package_dir = '/home/reynaldy/ros2_ws/src/my_nav2_bringup' # which works doesn't amtter where the path on the terminal is
    current_package_dir = os.path.join('src', 'my_nav2_bringup')


    # Declare and launch configuration
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(current_package_dir, 'maps', 'world.yaml'),
        description='Full path to map yaml file to load')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav2_bringup_dir, 'params', 'custom_params.yaml'), # for this we can still use install dir since params folder also get inside. But, rviz and maps need to use absolute src path
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_bt_file_cmd = DeclareLaunchArgument(
        'bt_file',
        default_value=os.path.join(nav2_bringup_dir, 'params', 'navigate_to_pose_w_replanning_and_recovery.xml'),
        description='Full path to the custom BT file to use for the bt_navigator'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration("map")
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    bt_file = LaunchConfiguration('bt_file')

    log_bt_file = LogInfo(msg=['Using BT file: ', bt_file])
    
    # bring up launch
    bring_up_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'map': map_yaml,
                'params_file': params_file,
                'bt_file': bt_file
            }.items())
    

    # colliison monitor launch
    colliison_monitor_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_collision_monitor_dir, 'launch', 'collision_monitor_node.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file
            }.items())

    # Rviz Node
    rviz_config_file_path = os.path.join(current_package_dir, 'rviz', 'my_rviz_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file_path]
    )

    ld = LaunchDescription()

    # Add Declaration
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_file_cmd)

    ld.add_action(log_bt_file)

    # Add all nodes and launch files
    ld.add_action(bring_up_cmd)
    ld.add_action(colliison_monitor_cmd)
    ld.add_action(rviz_node)

    return ld