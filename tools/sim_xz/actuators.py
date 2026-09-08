"""Delayed, asymmetric first-order actuator response; no control equations."""
from collections import deque
import math


def lag(actual, target, duration, tau):
    if tau <= 0:
        return target
    return actual + (target-actual)*(-math.expm1(-duration/tau))


class TiltActuator:
    def __init__(self, gain=1., delay=0., tau_increase=.08, tau_decrease=.08,
                 pulse_direction=1., limit=math.pi/2):
        self.gain, self.delay = gain, delay
        self.tau_increase, self.tau_decrease = tau_increase, tau_decrease
        self.pulse_direction, self.limit = pulse_direction, limit
        self.reset()

    def reset(self, actual=0.):
        self.pending = deque()
        self.target = actual
        self.command = None

    def _integrate(self, start, end, actual):
        def evolve(value, target, duration):
            increasing = (target-value)*self.pulse_direction >= 0
            tau = self.tau_increase if increasing else self.tau_decrease
            return lag(value, target, duration, tau)
        cursor=start
        while self.pending and self.pending[0][0] <= end+1e-12:
            when, target=self.pending.popleft()
            when=min(end, max(cursor, when))
            actual=evolve(actual,self.target,when-cursor)
            cursor=when
            self.target=target
        return evolve(actual,self.target,end-cursor)

    def step(self, actual, command, now, dt):
        command=max(-self.limit,min(self.limit,command))
        if command != self.command:
            self.pending.append((now+self.delay,command*self.gain))
            self.command=command
        mid=self._integrate(now,now+dt/2,actual)
        end=self._integrate(now+dt/2,now+dt,mid)
        return mid,end
