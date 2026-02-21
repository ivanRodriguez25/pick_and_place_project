
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess, IncludeLaunchDescription

def generate_launch_description():

    ld = LaunchDescription()
    
    xacro_path = 'urdf/panda.urdf.xacro'
    
    robot_description = PathJoinSubstitution([
        get_package_share_directory('panda_description'),	
        #get_package_share_directory('panda_moveit_config'),	
        xacro_path
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',        
        parameters=[{
            'robot_description':Command(['xacro ', robot_description] )
        }]
    )

    workspace_path = os.environ.get('COLCON_PREFIX_PATH') or os.environ.get('AMENT_PREFIX_PATH')
    pkg_panda_description = workspace_path + "/panda_description/share"
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[pkg_panda_description]
    )

    # Spawn
    spawn_node = Node(package='ros_gz_sim', executable='create',
                 arguments=['-name', 'panda', '-topic', '/robot_description'], output='screen')

    
    ignition_gazebo_node = IncludeLaunchDescription( PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
                                        launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])])
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen'
    )

    load_eef_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'eef_controller'],
        output='screen'
    )


    ld.add_action( load_joint_state_broadcaster )
    ld.add_action( load_position_controller )
    ld.add_action( load_eef_controller )
    ld.add_action( ign_resource_path )
    ld.add_action( robot_state_publisher_node )
    ld.add_action( spawn_node )
    ld.add_action( ignition_gazebo_node )

    return ld
