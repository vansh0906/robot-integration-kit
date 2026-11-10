"""Run the ROS 2 base driver node against stubbed rclpy (no ROS install needed).
The real launch_testing test runs in CI on ros:jazzy."""
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
STUB = ROOT / "tests" / "rosstub"
PKG = ROOT / "ros2_ws" / "src" / "rik_ros"
STUBBED = ("rclpy", "builtin_interfaces", "geometry_msgs", "std_msgs", "sensor_msgs", "nav_msgs",
           "rosgraph_msgs", "visualization_msgs", "tf2_ros", "rik_ros", "install", "rosstub_core")


@pytest.fixture
def ros():
    sys.path[:0] = [str(STUB), str(PKG)]
    import install  # noqa: F401
    from rosstub_core import GRAPH
    GRAPH.__init__()
    yield GRAPH
    sys.path.remove(str(STUB))
    sys.path.remove(str(PKG))
    for k in list(sys.modules):
        if k.split(".")[0] in STUBBED:
            del sys.modules[k]


@pytest.mark.parametrize("partner", ["partner_a_warehouse_amr", "partner_b_yard_tug", "partner_c_mecanum_cart"])
def test_base_driver_node(ros, partner):
    Node = sys.modules["rclpy.node"].Node
    Node.overrides = {"config_file": str(ROOT / "configs" / "partners" / f"{partner}.yaml"), "simulate": True}
    from rik_ros.base_driver_node import BaseDriverNode
    Twist = sys.modules["geometry_msgs.msg"].Twist
    Bool = sys.modules["std_msgs.msg"].Bool
    Empty = sys.modules["std_msgs.msg"].Empty
    node = BaseDriverNode()
    st, wheels = {}, []
    ros.subs.setdefault("/base_status", []).append(lambda m: st.update(json.loads(m.data)))
    ros.subs.setdefault("/wheel_cmd", []).append(lambda m: wheels.append(list(m.data)))

    def run(secs, vx=None):
        for _ in range(int(secs * 50)):
            ros.now += 0.02
            if vx is not None:
                t = Twist()
                t.linear.x, t.linear.y, t.angular.z = vx, 0.0, 0.0
                ros.publish("/cmd_vel", t)
            node.tick()

    run(3.0, vx=0.5)
    assert st["speed"] > 0.45 and st["x"] > 1.0
    assert len(wheels[-1]) == node.pf.n_act
    run(4.0)                                             # silence -> watchdog stop
    assert st["speed"] < 0.01 and "WATCHDOG" in st["events"]
    run(1.0, vx=0.5)
    ros.publish("/estop", Bool(data=True))
    run(3.0, vx=0.5)
    assert st["estop_latched"] and st["speed"] < 0.01
    ros.publish("/estop", Bool(data=False))
    run(1.0, vx=0.5)
    assert st["speed"] < 0.01                           # no motion before an explicit reset
    ros.publish("/reset", Empty())
    run(3.0, vx=0.5)
    assert st["speed"] > 0.45
