#!/usr/bin/python

import time
import numpy as np


class PID:
    def __init__(
        self,
        Kp=0.0,
        Ki=0.0,
        Kd=0.0,
        set_point=0.0,
        sample_time=0.01,
        out_limits=(None, None),
    ):

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0

        self.windup_guard = 20.0

        self.set_point = set_point

        self.sample_time = sample_time

        self.out_limits = out_limits

        self.last_err = 0.0

        self.last_time = time.time()

        self.output = 0.0

    def update(self, feedback_val):
        """Compute PID control value based on feedback_val.
        """

        # TODO: implement PID control
        error = self.set_point - feeback_val
        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.p_term = self.Kp * error
            self.i_term += error * delta_time

            if (self.i_term < -self.windup_guard):
                self.i_term = -self.windup_guard
            elif (self.i_term > self.windup_guard):
                self.i_term = self.windup_guard

            self.d_term = 0.0
            if delta_time > 0:
                self.d_term = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.p_term + (self.Ki * self.i_term) + (self.Kd * self.d_term)

    def __call__(self, feeback_val):
        return self.update(feeback_val)
