"""launch_testing: drive the base for each partner config; the watchdog and E-stop must stop it."""
import json
import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty, String

CFG = os.path.join(get_package_share_directory("rik_ros"), "configs", "partner_a_warehouse_amr.yaml")


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(package="rik_ros", executable="base_driver", output="screen",
                                   parameters=[{"config_file": CFG, "simulate": True}])
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


class TestBaseDriver(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("probe")
        self.st = {}
        self.node.create_subscription(String, "/base_status", lambda m: self.st.update(json.loads(m.data)), 10)
        self.cmd = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.estop = self.node.create_publisher(Bool, "/estop", 10)
        self.reset = self.node.create_publisher(Empty, "/reset", 10)

    def tearDown(self):
        self.node.destroy_node()

    def spin_for(self, secs, vx=None):
        end = time.time() + secs
        while time.time() < end:
            if vx is not None:
                t = Twist()
                t.linear.x = vx
                self.cmd.publish(t)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def test_drive_watchdog_estop(self):
        self.spin_for(3.0)                                   # discovery
        self.spin_for(3.0, vx=0.8)
        self.assertGreater(self.st.get("speed", 0), 0.7, self.st)
        x_moving = self.st["x"]
        self.assertGreater(x_moving, 1.0)
        self.spin_for(2.5)                                   # commands stop -> watchdog
        self.assertLess(self.st["speed"], 0.02, self.st)
        self.assertIn("WATCHDOG", self.st["events"])
        self.spin_for(2.0, vx=0.8)
        self.estop.publish(Bool(data=True))
        self.spin_for(2.0, vx=0.8)
        self.assertTrue(self.st["estop_latched"])
        self.assertLess(self.st["speed"], 0.02, self.st)
        self.estop.publish(Bool(data=False))
        self.spin_for(1.0, vx=0.8)
        self.assertLess(self.st["speed"], 0.02, "must not move before an explicit reset")
        self.reset.publish(Empty())
        self.spin_for(2.0, vx=0.8)
        self.assertGreater(self.st["speed"], 0.5, self.st)
