"""Register the stub modules in sys.modules."""
import sys
from rosstub_core import GRAPH, make, module

Time = make("Time")
Time.__init__ = lambda self, sec=0, nanosec=0: (setattr(self, "sec", sec), setattr(self, "nanosec", nanosec), None)[-1]


class _Param:
    def __init__(self, v):
        self.value = v


class _Pub:
    def __init__(self, topic):
        self.topic = topic

    def publish(self, msg):
        GRAPH.publish(self.topic, msg)


class _Now:
    def __init__(self, t):
        self.nanoseconds = int(round(t * 1e9))

    def to_msg(self):
        return Time(sec=int(self.nanoseconds // 1_000_000_000), nanosec=int(self.nanoseconds % 1_000_000_000))


class _Clock:
    def now(self):
        return _Now(GRAPH.now)


class _Logger:
    def info(self, s):
        print("[ros]", s)

    warn = error = info


class Node:
    overrides = {}

    def __init__(self, name):
        self.name = name
        self._params = {}

    def declare_parameter(self, name, default):
        self._params[name] = Node.overrides.get(name, default)

    def get_parameter(self, name):
        return _Param(self._params[name])

    def create_publisher(self, typ, topic, qos):
        return _Pub(topic)

    def create_subscription(self, typ, topic, cb, qos):
        GRAPH.subs.setdefault(topic, []).append(cb)

    def create_timer(self, period, cb):
        GRAPH.timers.append([period, GRAPH.now + period, cb, self.name])

    def get_clock(self):
        return _Clock()

    def get_logger(self):
        return _Logger()

    def destroy_node(self):
        pass


rclpy = module("rclpy", init=lambda *a, **k: None, ok=lambda: True, shutdown=lambda: None, spin=lambda n: None)
rclpy.node = module("rclpy.node", Node=Node)
sys.modules["rclpy"] = rclpy
sys.modules["rclpy.node"] = rclpy.node

M = {
    "builtin_interfaces.msg": dict(Time=Time),
    "geometry_msgs.msg": dict(Quaternion=make("Quaternion"), PoseWithCovarianceStamped=make("PoseWithCovarianceStamped"),
                              PoseStamped=make("PoseStamped"), TransformStamped=make("TransformStamped"),
                              Point=make("Point"), Twist=make("Twist")),
    "std_msgs.msg": dict(UInt8MultiArray=make("UInt8MultiArray"), String=make("String"), Empty=make("Empty"),
                         ColorRGBA=make("ColorRGBA"), Bool=make("Bool"), Float64MultiArray=make("Float64MultiArray")),
    "sensor_msgs.msg": dict(Imu=make("Imu"), LaserScan=make("LaserScan"),
                            NavSatFix=make("NavSatFix", COVARIANCE_TYPE_DIAGONAL_KNOWN=2),
                            NavSatStatus=make("NavSatStatus", STATUS_NO_FIX=-1, STATUS_FIX=0, STATUS_SBAS_FIX=1,
                                              STATUS_GBAS_FIX=2)),
    "nav_msgs.msg": dict(Odometry=make("Odometry"), Path=make("Path")),
    "rosgraph_msgs.msg": dict(Clock=make("Clock")),
    "visualization_msgs.msg": dict(Marker=make("Marker", ADD=0, DELETE=2, DELETEALL=3, CUBE=1, SPHERE=2, CYLINDER=3,
                                               LINE_LIST=5), MarkerArray=make("MarkerArray")),
}
for name, attrs in M.items():
    pkg = name.split(".")[0]
    sys.modules.setdefault(pkg, module(pkg))
    sys.modules[name] = module(name, **attrs)


class _Tf:
    def __init__(self, node):
        pass

    def sendTransform(self, t):
        pass


sys.modules["tf2_ros"] = module("tf2_ros", TransformBroadcaster=_Tf, StaticTransformBroadcaster=_Tf)
