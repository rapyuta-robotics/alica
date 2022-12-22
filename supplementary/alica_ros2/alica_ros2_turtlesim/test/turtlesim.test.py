import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest

from ament_index_python import get_package_share_directory


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    # Create a LaunchDescription and add nodes
    ld = launch.LaunchDescription()

    master_plan = launch.substitutions.LaunchConfiguration(
        'master_plan', default='Master')
    roleset = launch.substitutions.LaunchConfiguration(
        'roleset', default='Roleset')
    alica_path = launch.substitutions.LaunchConfiguration(
        'alica_path', default=get_package_share_directory('alica_ros2_turtlesim'))

    turtlesim_node = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim'
    )
    ld.add_action(turtlesim_node)

    # Add the alica_ros2_turtlesim nodes
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
    ld.add_action(launch_testing.actions.ReadyToTest())

    return ld

# After all these tests are done, the launch system will shut down the processes that it started up


class TestTurtlesAtTarget(unittest.TestCase):
    def test_turtles_at_target(self, proc_output):
        proc_output.assertWaitFor(
            'Turtle turtle0 reached target', timeout=15, stream='stderr')
        proc_output.assertWaitFor(
            'Turtle turtle1 reached target', timeout=15, stream='stderr')
        proc_output.assertWaitFor(
            'Turtle turtle2 reached target', timeout=15, stream='stderr')
        proc_output.assertWaitFor(
            'Turtle turtle3 reached target', timeout=15, stream='stderr')


class TestTurtlesSpawn(unittest.TestCase):
    def test_turtles_spawn(self, proc_output):
        proc_output.assertWaitFor(
            'Spawning turtle [turtle0] at', timeout=15, stream='stderr')
        proc_output.assertWaitFor(
            'Spawning turtle [turtle1] at', timeout=15, stream='stderr')
        proc_output.assertWaitFor(
            'Spawning turtle [turtle2] at', timeout=15, stream='stderr')
        proc_output.assertWaitFor(
            'Spawning turtle [turtle1] at', timeout=15, stream='stderr')


class TestTurtlesTeleport(unittest.TestCase):
    def test_turtles_teleport(self, proc_output):
        proc_output.assertWaitFor(
            '[turtle0]: teleport req to x:', timeout=15, stream='stderr')
        proc_output.assertWaitFor(
            '[turtle1]: teleport req to x:', timeout=15, stream='stderr')
        proc_output.assertWaitFor(
            '[turtle2]: teleport req to x:', timeout=15, stream='stderr')
        proc_output.assertWaitFor(
            '[turtle3]: teleport req to x:', timeout=15, stream='stderr')
