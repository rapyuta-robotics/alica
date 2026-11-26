# Runs the ALICA turtle node against turtlesim_node through its whole life: it respawns its turtle, drives it to the center
# after /init, and exits cleanly on SIGINT. The node used to crash right after the respawn, and again when the engine terminated.
import math
import os
import signal
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from std_msgs.msg import Empty
from turtlesim.msg import Pose

TURTLE = 'turtle1'
# The leader's goal in CircleRuntimeConditionConstraint, with some margin over its tolerance of 0.1
CENTER_X, CENTER_Y = 5.0, 5.0
CENTER_TOLERANCE = 0.3


@pytest.mark.launch_test
def generate_test_description():
    turtlesim = launch_ros.actions.Node(
        package='turtlesim', executable='turtlesim_node', additional_env={'QT_QPA_PLATFORM': 'offscreen'})
    # As in turtle.launch.xml
    alica = launch_ros.actions.Node(
        package='alica_ros2_turtlesim',
        executable='alica_ros2_turtlesim',
        name=TURTLE,
        output='screen',
        parameters=[{
            'name': TURTLE,
            'agent_id': 1,
            'master_plan': 'Master',
            'roleset': 'Roleset',
            'alica_path': get_package_share_directory('alica_ros2_turtlesim'),
        }])
    return launch.LaunchDescription([turtlesim, alica, launch_testing.actions.ReadyToTest()]), {'alica': alica}


class TestTurtle(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('turtle_launch_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_spawn_drive_and_stop(self, alica, proc_info, proc_output):
        # The node used to crash right after this
        proc_output.assertWaitFor(f'Spawned {TURTLE}', process=alica, timeout=30)
        # Go2RandomPosition teleports the turtle and then clears the /init trigger, so publish /init after this
        proc_output.assertWaitFor('teleport req', process=alica, timeout=30)

        poses = []
        velocities = []
        self.node.create_subscription(Pose, f'/{TURTLE}/pose', poses.append, 10)
        self.node.create_subscription(Twist, f'/{TURTLE}/cmd_vel', velocities.append, 10)
        init = self.node.create_publisher(Empty, '/init', 1)

        # On /init the plan switches to GoTo, which drives the only turtle, the leader, to the center. Publish /init until the
        # turtle drives, in case the node has not discovered this publisher yet.
        def at_center():
            return bool(poses) and math.hypot(poses[-1].x - CENTER_X, poses[-1].y - CENTER_Y) < CENTER_TOLERANCE

        deadline = time.monotonic() + 30
        next_init = 0.0
        while not (velocities and at_center()) and time.monotonic() < deadline:
            if not velocities and time.monotonic() >= next_init:
                init.publish(Empty())
                next_init = time.monotonic() + 1
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(velocities, f'{TURTLE} did not drive after /init')
        self.assertTrue(at_center(), f'{TURTLE} did not reach the center; last pose: {poses[-1] if poses else None}')

        # Stop the node while turtlesim still runs, so that it can kill its turtle on the way out
        os.kill(alica.process_details['pid'], signal.SIGINT)
        proc_info.assertWaitForShutdown(process=alica, timeout=30)
        proc_output.assertWaitFor(f'Request to Kill: {TURTLE} complete', process=alica, timeout=1)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, alica, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, process=alica)
