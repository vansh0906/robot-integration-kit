"""Base driver: the layer between the navigation stack and the platform's motor controllers.

The same class runs in the simulator and inside the ROS 2 node (ros2_ws/src/rik_ros).
- Command watchdog: no velocity command for `watchdog_timeout` -> ramp to a stop.
- E-stop: latched; motors commanded to zero at once; needs an explicit reset after release.
- Deceleration ramp so a stop never exceeds the platform's rated decel.
"""
import numpy as np

WATCHDOG_TIMEOUT = 0.25


class BaseDriver:
    def __init__(self, platform, watchdog_timeout=WATCHDOG_TIMEOUT):
        self.pf = platform
        self.timeout = watchdog_timeout
        self.t_last_cmd = -1e9
        self.cmd = np.zeros(3)
        self.out = np.zeros(3)
        self.estop = False
        self.estop_latched = False
        self.watchdog_tripped = False
        self.events = []

    def on_cmd(self, t, cmd):
        self.cmd = np.asarray(cmd, float)
        self.t_last_cmd = t

    def press_estop(self, t):
        self.estop = self.estop_latched = True
        self.events.append((t, "ESTOP"))

    def release_estop(self, t):
        self.estop = False
        self.events.append((t, "ESTOP_RELEASED"))

    def reset(self, t):
        if not self.estop:
            self.estop_latched = False
            self.events.append((t, "RESET"))

    def update(self, t, dt):
        """Returns actuator targets for this control tick."""
        if self.estop_latched:
            self.out = np.zeros(3)
            return self.pf.command(self.out) * 0.0
        stale = t - self.t_last_cmd > self.timeout
        if stale and not self.watchdog_tripped:
            self.watchdog_tripped = True
            self.events.append((t, "WATCHDOG"))
        elif not stale and self.watchdog_tripped:
            self.watchdog_tripped = False
            self.events.append((t, "WATCHDOG_CLEARED"))
        target = np.zeros(3) if stale else self.pf.limit_body(self.cmd)
        # never decelerate harder than the platform is rated for
        dv = target - self.out
        lin = np.hypot(*dv[:2])
        lim = self.pf.decel * dt
        if lin > lim and np.hypot(*target[:2]) < np.hypot(*self.out[:2]):
            dv[:2] *= lim / lin
        self.out = self.out + dv
        return self.pf.command(self.out)
