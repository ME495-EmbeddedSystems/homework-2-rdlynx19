"""ROS Test to verify cmd_vel commands are published at 100Hz."""
import time
import unittest

from geometry_msgs.msg import Twist

from launch import LaunchDescription

from launch_ros.actions import Node

from launch_testing.actions import ReadyToTest

import launch_testing_ros

import pytest

import rclpy


# Mark the launch description generation as a rostest
# Also, it's called generate_test_description() for a test
# But it still returns a LaunchDescription
@pytest.mark.rostest
def generate_test_description():
    """Generate launch description for test launch file."""
    # Create a node, as usual. But it's useful to keep it around
    node = Node(package='turtle_brick', executable='turtle_robot')
    # return a tuple (LaunchDescription, extra_dict)
    return (
        LaunchDescription([
            node,
            ReadyToTest()  # this is the last action. Can be used elsewhere
                ]),
        {'myaction': node}
                )
# The above returns the launch description. Now it's time for the test
# call services and subscribe/publish messages
# unlike a regular node, it is often useful to not subclass node but rather
# just create it. It is also useful (and necessary) to spin_once() as needed


class TestMyVelocity(unittest.TestCase):
    """unittest subclass to test the cmd_vel command rate."""

    @classmethod
    def setUpClass(cls):
        """Run one time, when the testcase is loaded."""
        rclpy.init()
        cls.pub_counter = 0

    @classmethod
    def tearDownClass(cls):
        """Run one time, when testcase is unloaded."""
        rclpy.shutdown()

    def setUp(self):
        """Run before every test."""
        # so before every test, we create a new node
        self.node = rclpy.create_node('test_node')
        self.vel_sub = self.node.create_subscription(
            Twist, 'cmd_vel', self.vel_callback, 1)

    def tearDown(self):
        """Run after every test."""
        # so after every test we destroy the node
        # Is a new node necessary for each test, or could we
        # create the nodes when we setupClass?
        self.node.destroy_node()

    def test_my_test1(self, launch_service, myaction, proc_output):
        """
        In UnitTest, any function starting with "test" is run as a test.

        Args:
        ----
        launch_service: information about the launch
        myaction: this was passed in when we created the description
        proc_output: this object streams the output (if any)
        from the running process

        """
        # ----------------- Begin_Citation [6] --------------- #
        available_topics = launch_testing_ros.WaitForTopics([(
            'cmd_vel', Twist)], timeout=10.0)
        available_topics.wait()
        pre_time = time.time()
        while (self.pub_counter < 100):
            rclpy.spin_once(self.node)
            if (time.time() - pre_time > 2.0):
                self.fail('The test failed')
        post_time = time.time()
        # ------------------ End_Citation [6] --------------- #
        # ------------------ Begin_Citation [7] -------------- #
        self.assertEqual(post_time - pre_time > 0.9, True)
        self.assertEqual(post_time - pre_time < 1.1, True)
        # ------------------ End_Citation [7] ---------------- #
        

    def vel_callback(self, vel_msg):
        """Increase counter variable by one, when a msg is received."""
        self.pub_counter = self.pub_counter + 1
