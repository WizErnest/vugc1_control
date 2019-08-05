#!/usr/bin/env python


class PIDcontroller(object):

    def __init__(self, kp=0, ki=0, kd=0, limited=False, ub=0, lb=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.correction = 0
        self.ub = ub
        self.lb = lb
        self.limited = limited

        self.err_cur = 0
        self.err_prv = 0
        self.err_sum = 0
        self.err_dif = 0

    def set_pid(self, kp=0, ki=0, kd=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def update_error(self, setpoint=0, measurement=0, dt=1):
        self.err_prv = self.err_cur
        self.err_cur = setpoint - measurement
        self.err_sum += self.err_cur * dt
        self.err_dif = 0 if dt == 0 else (self.err_cur - self.err_prv)/dt

    def clamped(self):
        # checks if integrator should be clamped based on past output and current error
        saturated = self.correction == self.ub or self.correction == self.lb
        samesign = ((self.correction < 0) == (self.err_cur < 0))

        return (saturated and samesign) and self.limited

    def calculate_correction(self):
        p_val = self.kp * self.err_cur
        i_val = 0 if self.clamped() else self.ki * self.err_sum
        d_val = self.kd * self.err_dif

        self.correction = p_val + i_val + d_val
        self.correction = max(min(self.correction, self.ub), self.lb) if self.limited else self.correction

    def update(self, setpoint=0, measurement=0, dt=1):
        self.update_error(setpoint, measurement, dt)
        self.calculate_correction()
