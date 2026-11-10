"""ROS 2 base driver node: the kit's BaseDriver (watchdog, latched E-stop, decel ramp) in front of the
partner's motor controllers.

Subscribes: /cmd_vel (geometry_msgs/Twist), /estop (std_msgs/Bool, true = pressed), /reset (std_msgs/Empty)
Publishes:  /wheel_cmd (std_msgs/Float64MultiArray: wheel rad/s, or [wheel rad/s, steer rad] for Ackermann)
            /odom (nav_msgs/Odometry from the wheel states), /base_status (std_msgs/String, JSON)
With simulate:=true the node also runs the platform's actuator model, so the loop can be tested
without hardware.
"""
import json
import math
import os

try:
    import rik  # noqa: F401
except ImportError:
    import sys
    if os.environ.get("RIK_ROOT"):
        sys.path.insert(0, os.environ["RIK_ROOT"])

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, Float64MultiArray, String

from rik import config
from rik.driver import BaseDriver
from rik.platforms import make_platform

RATE = 50.0


class BaseDriverNode(Node):
    def __init__(self):
        super().__init__("rik_base_driver")
        self.declare_parameter("config_file", "")
        self.declare_parameter("simulate", True)
        self.declare_parameter("watchdog_timeout", 0.25)
        path = self.get_parameter("config_file").value
        cfg = config.load(path)
        errs = config.errors(config.lint(cfg))
        if errs:
            for e in errs:
                self.get_logger().error(str(e))
            raise RuntimeError(f"config {path} has {len(errs)} lint errors")
        self.pf = make_platform(cfg["platform"])
        self.drv = BaseDriver(self.pf, float(self.get_parameter("watchdog_timeout").value))
        self.simulate = bool(self.get_parameter("simulate").value)
        self.pose = np.zeros(3)
        self.t0 = self.now()
        self.p_wheels = self.create_publisher(Float64MultiArray, "/wheel_cmd", 10)
        self.p_odom = self.create_publisher(Odometry, "/odom", 10)
        self.p_status = self.create_publisher(String, "/base_status", 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(Bool, "/estop", self.on_estop, 10)
        self.create_subscription(Empty, "/reset", self.on_reset, 10)
        self.create_timer(1.0 / RATE, self.tick)
        self.k = 0
        self.get_logger().info(f"{cfg.get('partner', path)}: {self.pf.kind}, watchdog {self.drv.timeout} s")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, msg):
        self.drv.on_cmd(self.now(), [msg.linear.x, msg.linear.y, msg.angular.z])

    def on_estop(self, msg):
        if msg.data and not self.drv.estop:
            self.drv.press_estop(self.now())
        elif not msg.data and self.drv.estop:
            self.drv.release_estop(self.now())

    def on_reset(self, _):
        self.drv.reset(self.now())

    def tick(self):
        t = self.now()
        dt = 1.0 / RATE
        target = self.drv.update(t, dt)
        self.p_wheels.publish(Float64MultiArray(data=[float(v) for v in target]))
        if self.simulate:
            true_tw, meas = self.pf.step(target, dt)
        else:
            meas = self.pf.from_actuators(target)
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        self.pose += np.array([(c * meas[0] - s * meas[1]) * dt, (s * meas[0] + c * meas[1]) * dt, meas[2] * dt])
        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id, od.child_frame_id = "odom", "base_link"
        od.pose.pose.position.x, od.pose.pose.position.y = float(self.pose[0]), float(self.pose[1])
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = math.sin(self.pose[2] / 2), math.cos(self.pose[2] / 2)
        od.twist.twist.linear.x, od.twist.twist.linear.y = float(meas[0]), float(meas[1])
        od.twist.twist.angular.z = float(meas[2])
        self.p_odom.publish(od)
        self.k += 1
        if self.k % 5 == 0:
            st = {"t": round(t - self.t0, 3), "estop_latched": self.drv.estop_latched, "estop": self.drv.estop,
                  "watchdog": self.drv.watchdog_tripped, "speed": float(math.hypot(meas[0], meas[1])),
                  "x": float(self.pose[0]), "events": [e[1] for e in self.drv.events][-5:]}
            self.p_status.publish(String(data=json.dumps(st)))


def main():
    rclpy.init()
    node = BaseDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
