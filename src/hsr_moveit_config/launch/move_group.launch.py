import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_xacro(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    doc = xacro.process_file(absolute_file_path).toprettyxml(indent='  ')
    return doc


def generate_launch_description():
    declared_arguments = [DeclareLaunchArgument("robot_ip", default_value="192.170.10.2",
                                                description="IP address of the kuka KONI"),
                          DeclareLaunchArgument("robot_port", default_value="30200",
                                                description="Port used by the FRI (30200 - 30209"),
                          DeclareLaunchArgument("real", default_value="false",
                                                description="Type of manipulator to startup (fake/false or real/true)")]

    robot_ip = LaunchConfiguration("robot_ip")
    robot_port = LaunchConfiguration("robot_port")
    manipulator = LaunchConfiguration("real")

    nodes = []
    # Command-line arguments
    walls_arg = DeclareLaunchArgument(
        "walls", default_value="True", description="Walls flag"
    )
    nodes.append(walls_arg)

    # Component yaml files are grouped in separate namespaces
    ######################
    #### Config Files ####
    ######################
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare('hsr_support'), "urdf", 'iiwa_workcell.urdf.xacro']),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            " ",
            "robot_port:=",
            robot_port,
            " ",
            " ",
            "hardware:=",
            manipulator,
            " "
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_description_semantic_config = load_file('hsr_moveit_config', 'config/iiwa_workcell.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('hsr_moveit_config', 'config/kinematics_kdl.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # Planning Functionality
    ompl_planning_pipeline_config = {'move_group': {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
        'start_state_max_bounds_error': 0.1}}
    ompl_planning_yaml = load_yaml('hsr_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    controllers_yaml = load_yaml('hsr_moveit_config', 'config/controllers.yaml')
    moveit_controllers = {'moveit_simple_controller_manager': controllers_yaml,
                          'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}

    trajectory_execution = {'moveit_manage_controllers': True,
                            'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                            'trajectory_execution.allowed_goal_duration_margin': 0.5,
                            'trajectory_execution.allowed_start_tolerance': 0.01}

    planning_scene_monitor_parameters = {"publish_planning_scene": True,
                                         "publish_geometry_updates": True,
                                         "publish_state_updates": True,
                                         "publish_transforms_updates": True,
                                         "publish_robot_description": True,
                                         "publish_robot_description_semantic": True}

    
    # Start the actual move_group node/action server
    move_group_node = Node(package='moveit_ros_move_group',
                           executable='move_group',
                           output='screen',
                           parameters=[robot_description,
                                       robot_description_semantic,
                                       kinematics_yaml,
                                       ompl_planning_pipeline_config,
                                       trajectory_execution,
                                       moveit_controllers,
                                       planning_scene_monitor_parameters
                                       ])
    nodes.append(move_group_node)

    # RViz
    rviz_config_file = get_package_share_directory('hsr_moveit_config') + "/launch/moveit_iiwa_config.rviz"

    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description,
                                 robot_description_semantic,
                                 ompl_planning_pipeline_config,
                                 robot_description_kinematics,
                                 ]
                     )
    nodes.append(rviz_node)

    # Publish TF

    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='both',
                                 parameters=[robot_description])
    nodes.append(robot_state_publisher)

    # Iiwa settings
    iiwa_controller = os.path.join(
        get_package_share_directory('hsr_moveit_config'),
        'config',
        'ros_controllers.yaml'
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, iiwa_controller],
        prefix=['nice -n -20 '],
        output="both"
    )
    nodes.append(ros2_control_node)

    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_controller", "-c", "/controller_manager"],
    )
    nodes.append(joint_state_broadcaster_spawner)

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["iiwa_arm_controller", "-c", "/controller_manager"],
    )
    nodes.append(robot_controller_spawner)
   
     
    hsr_scene_geometry = Node(
        package='hsr_scene_geometry',
        executable='hsr_scene_geometry',
        name='hsr_scene_geometry',
        condition=IfCondition(LaunchConfiguration("walls")),
    )
    nodes.append(hsr_scene_geometry)

    return LaunchDescription(declared_arguments + nodes)
