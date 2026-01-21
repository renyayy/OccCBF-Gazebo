import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('occlusion_sim')
    urdf = os.path.join(pkg, 'urdf', 'simple_holonomic_robot.urdf')
    world = os.path.join(pkg, 'worlds', 'multi_obstacle.world')

    obstacles = [
        {'name': 'obs_0', 'x': 3.0, 'y': -2.0},
        {'name': 'obs_1', 'x': 5.0, 'y': 1.0},
        {'name': 'obs_2', 'x': 4.0, 'y': -1.0},
        {'name': 'obs_3', 'x': 2.0, 'y': 3.0},
        {'name': 'obs_4', 'x': 6.0, 'y': -3.0},
    ]

    ld = LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-entity', 'ego_robot', '-file', urdf, '-x', '0', '-y', '0', '-z', '0.2'],
             output='screen'),
    ])

    # Spawn multiple dynamic obstacles
    for obs in obstacles:
        sdf = os.path.join(pkg, 'models', 'dynamic_obstacle.sdf')
        with open(sdf, 'r') as f:
            sdf_content = f.read().replace('/obs_{id}', f'/{obs["name"]}')
        tmp_sdf = f'/tmp/{obs["name"]}.sdf'
        with open(tmp_sdf, 'w') as f:
            f.write(sdf_content)
        ld.add_action(Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-entity', obs['name'], '-file', tmp_sdf,
                       '-x', str(obs['x']), '-y', str(obs['y']), '-z', '0.1'],
            output='screen'))

    ld.add_action(Node(package='occlusion_sim', executable='multi_obstacle_controller.py', output='screen'))
    ld.add_action(Node(package='occlusion_sim', executable='cbf_wrapper_node.py', output='screen'))
    return ld
