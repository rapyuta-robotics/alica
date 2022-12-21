import launch
import launch_ros.actions


def generate_launch_description():
    # Create a LaunchDescription and add nodes
    ld = launch.LaunchDescription()
    # Parse command line arguments
    master_plan = launch.substitutions.LaunchConfiguration(
        'master_plan', default='Master')
    roleset = launch.substitutions.LaunchConfiguration(
        'roleset', default='Roleset')
    alica_path = launch.substitutions.LaunchConfiguration(
        'alica_path', default='/home/rapyuta/alica/install/alica_ros2_turtlesim/share/alica_ros2_turtlesim')  # fix me
    turtles = launch.substitutions.LaunchConfiguration('turtles', default='1')

    turtlesim_node = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim'
    )
    ld.add_action(turtlesim_node)
    # Add the alica_ros2_turtlesim node
    for i in range(4):
        agent_name = 'turtle' + str(3 - i)
        alica_ros2_turtlesim_node = launch_ros.actions.Node(
            package='alica_ros2_turtlesim',
            executable='alica_ros2_turtlesim',
            name=agent_name,
            parameters=[{'name': agent_name},
                        {'agent_id': 3-i},
                        {'master_plan': master_plan},
                        {'roleset': roleset},
                        {'alica_path': alica_path}]
        )
        ld.add_action(alica_ros2_turtlesim_node)
    start_sim = launch.actions.ExecuteProcess(
        cmd=[[
            launch.substitutions.FindExecutable(name='ros2'),
            ' topic pub ',
            '/init ',
            'std_msgs/msg/Empty ',
            '"{}"'
        ]],
        shell=True
    )
    ld.add_action(start_sim)
    return ld
