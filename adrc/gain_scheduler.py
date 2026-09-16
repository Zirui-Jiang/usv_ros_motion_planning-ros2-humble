"""Neural-network gain scheduler for heading control (Simulink Python block)."""

import math


def _wrap_to_pi(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def _saturate(value, lower, upper):
    return min(max(float(value), float(lower)), float(upper))


class NNGainScheduler:
    def __init__(self, adaptation_rate=0.02):
        self.adaptation_rate = float(adaptation_rate)
        self.adaptation = 0.0

    def reset(self):
        self.adaptation = 0.0

    def _compute(self, desire_psi, measured_psi, r, acc):
        e = _wrap_to_pi(float(measured_psi) - float(desire_psi))
        x1 = _saturate(e / math.pi, -1.0, 1.0)
        x2 = _saturate(float(r) / 1.5, -1.0, 1.0)
        x3 = _saturate(float(acc) / 4.0, -1.0, 1.0)

        m1 = abs(x1)
        m2 = abs(x2)
        m3 = abs(x3)

        h1 = math.tanh(2.2 * m1 - 0.55 * m2 + 0.08 * m3)
        h2 = math.tanh(1.35 * m2 + 0.20 * m3)
        h3 = math.tanh(0.60 * m1 + 0.45 * m3)

        target = _saturate(m1 - 0.20 * m2, 0.0, 1.0)
        return h1, h2, h3, target

    def outputs(self, desire_psi, measured_psi, r, acc):
        h1, h2, h3, _ = self._compute(desire_psi, measured_psi, r, acc)
        ad = self.adaptation
        kp = _saturate(0.78 + 0.52 * h1 + 0.20 * ad, 0.65, 1.50)
        kd = _saturate(0.18 + 0.22 * h2 + 0.05 * h3, 0.12, 0.45)
        ki = _saturate(0.001 + 0.004 * ad, 0.0, 0.005)
        return kp, kd, ki

    def update(self, desire_psi, measured_psi, r, acc):
        _, _, _, target = self._compute(desire_psi, measured_psi, r, acc)
        a = self.adaptation + self.adaptation_rate * (target - self.adaptation)
        self.adaptation = _saturate(a, 0.0, 1.0)


_scheduler = None


def initialize_scheduler(adaptation_rate=0.02):
    global _scheduler
    _scheduler = NNGainScheduler(adaptation_rate)
    return 1


def reset_scheduler():
    if _scheduler is None:
        raise RuntimeError("Call initialize_scheduler before reset_scheduler")
    _scheduler.reset()
    return 1


def scheduler_outputs(desire_psi, measured_psi, r, acc):
    if _scheduler is None:
        raise RuntimeError("Call initialize_scheduler before scheduler_outputs")
    return _scheduler.outputs(desire_psi, measured_psi, r, acc)


def scheduler_update(desire_psi, measured_psi, r, acc):
    if _scheduler is None:
        raise RuntimeError("Call initialize_scheduler before scheduler_update")
    _scheduler.update(desire_psi, measured_psi, r, acc)
    return 1