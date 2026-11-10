"""Shared state for the stubbed ROS graph."""
import types


class Graph:
    def __init__(self):
        self.subs = {}
        self.timers = []         # [period, next_due, cb, uses_sim_time]
        self.now = 0.0
        self.published = {}

    def publish(self, topic, msg):
        self.published[topic] = self.published.get(topic, 0) + 1
        for cb in list(self.subs.get(topic, [])):
            cb(msg)


GRAPH = Graph()


class Msg:
    """Generic message: keyword fields, nested messages created on first access, list fields default to []."""
    LISTS = {"points", "markers", "poses", "data", "ranges", "covariance", "orientation_covariance",
             "position_covariance"}

    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        if name in Msg.LISTS:
            val = [0.0] * 9 if name.endswith("_covariance") else []
        else:
            val = Msg()
        object.__setattr__(self, name, val)
        return val

    def __lt__(self, other):          # an unset numeric field compares like 0
        return 0 < other


def make(name, **consts):
    return type(name, (Msg,), dict(consts))


def module(name, **attrs):
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    return m
