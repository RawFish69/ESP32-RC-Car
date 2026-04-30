"""Madgwick filter for optional MPU6050 IMU orientation fusion."""

from __future__ import annotations

import math

import numpy as np


class MadgwickFilter:
    def __init__(self, beta: float = 0.08) -> None:
        self.beta = beta
        self.reset()

    def reset(self) -> None:
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    def update(self, gyro_rads: np.ndarray, accel_mps2: np.ndarray, dt: float) -> np.ndarray:
        if dt <= 0.0:
            return self.q.astype(np.float32)

        gx, gy, gz = (float(v) for v in gyro_rads)
        ax, ay, az = (float(v) for v in accel_mps2)
        q1, q2, q3, q4 = self.q

        q_dot_1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz)
        q_dot_2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy)
        q_dot_3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx)
        q_dot_4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx)

        accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
        if accel_norm > 1.0e-9:
            ax /= accel_norm
            ay /= accel_norm
            az /= accel_norm

            f1 = 2.0 * (q2 * q4 - q1 * q3) - ax
            f2 = 2.0 * (q1 * q2 + q3 * q4) - ay
            f3 = 1.0 - 2.0 * (q2 * q2 + q3 * q3) - az

            j_11_or_24 = 2.0 * q3
            j_12_or_23 = 2.0 * q4
            j_13_or_22 = 2.0 * q1
            j_14_or_21 = 2.0 * q2
            j_32 = 2.0 * j_14_or_21
            j_33 = 2.0 * j_11_or_24

            s1 = j_14_or_21 * f2 - j_11_or_24 * f1
            s2 = j_12_or_23 * f1 + j_13_or_22 * f2 - j_32 * f3
            s3 = j_12_or_23 * f2 - j_33 * f3 - j_13_or_22 * f1
            s4 = j_14_or_21 * f1 + j_11_or_24 * f2

            step_norm = math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)
            if step_norm > 1.0e-9:
                s1 /= step_norm
                s2 /= step_norm
                s3 /= step_norm
                s4 /= step_norm

                q_dot_1 -= self.beta * s1
                q_dot_2 -= self.beta * s2
                q_dot_3 -= self.beta * s3
                q_dot_4 -= self.beta * s4

        q1 += q_dot_1 * dt
        q2 += q_dot_2 * dt
        q3 += q_dot_3 * dt
        q4 += q_dot_4 * dt

        quat_norm = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
        if quat_norm <= 1.0e-9:
            self.reset()
        else:
            self.q[:] = (q1 / quat_norm, q2 / quat_norm, q3 / quat_norm, q4 / quat_norm)

        return self.q.astype(np.float32)
