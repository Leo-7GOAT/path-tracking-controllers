import bisect
import math

import numpy as np


class Spline1D:
    def __init__(self, x, y):
        self.x = np.asarray(x, dtype=float)
        self.y = np.asarray(y, dtype=float)

        if np.any(np.diff(self.x) <= 0.0):
            raise ValueError("x coordinates must be strictly increasing for cubic spline.")

        self.nx = len(self.x)
        h = np.diff(self.x)

        self.a = self.y.copy()
        A = self._calc_A(h)
        B = self._calc_B(h)
        self.c = np.linalg.solve(A, B)
        self.b = np.zeros(self.nx - 1)
        self.d = np.zeros(self.nx - 1)

        for i in range(self.nx - 1):
            self.d[i] = (self.c[i + 1] - self.c[i]) / (3.0 * h[i])
            self.b[i] = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * (2.0 * self.c[i] + self.c[i + 1]) / 3.0

    def calc(self, t):
        if t < self.x[0] or t > self.x[-1]:
            return None

        i = self._search_index(t)
        dx = t - self.x[i]
        return self.a[i] + self.b[i] * dx + self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

    def calc_first_derivative(self, t):
        if t < self.x[0] or t > self.x[-1]:
            return None

        i = self._search_index(t)
        dx = t - self.x[i]
        return self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0

    def calc_second_derivative(self, t):
        if t < self.x[0] or t > self.x[-1]:
            return None

        i = self._search_index(t)
        dx = t - self.x[i]
        return 2.0 * self.c[i] + 6.0 * self.d[i] * dx

    def _search_index(self, x_value):
        return bisect.bisect(self.x, x_value) - 1

    def _calc_A(self, h):
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != self.nx - 2:
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]
        A[0, 1] = 0.0
        A[-1, -2] = 0.0
        A[-1, -1] = 1.0
        return A

    def _calc_B(self, h):
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / h[i + 1] - 3.0 * (
                self.a[i + 1] - self.a[i]
            ) / h[i]
        return B


class Spline2D:
    def __init__(self, x, y):
        self.s = self._calc_s(x, y)
        self.sx = Spline1D(self.s, x)
        self.sy = Spline1D(self.s, y)

    def _calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        ds = np.hypot(dx, dy)
        return np.concatenate(([0.0], np.cumsum(ds)))

    def calc_position(self, s):
        return self.sx.calc(s), self.sy.calc(s)

    def calc_yaw(self, s):
        dx = self.sx.calc_first_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        return math.atan2(dy, dx)

    def calc_curvature(self, s):
        dx = self.sx.calc_first_derivative(s)
        ddx = self.sx.calc_second_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        ddy = self.sy.calc_second_derivative(s)
        return (ddy * dx - ddx * dy) / ((dx * dx + dy * dy) ** 1.5 + 1e-9)


def calc_spline_course(x, y, ds=0.1):
    spline = Spline2D(x, y)
    s = np.arange(0.0, spline.s[-1], ds)

    rx = []
    ry = []
    ryaw = []
    rk = []
    for s_value in s:
        px, py = spline.calc_position(s_value)
        rx.append(px)
        ry.append(py)
        ryaw.append(spline.calc_yaw(s_value))
        rk.append(spline.calc_curvature(s_value))

    if not rx or rx[-1] != x[-1] or ry[-1] != y[-1]:
        rx.append(float(x[-1]))
        ry.append(float(y[-1]))
        ryaw.append(float(ryaw[-1] if ryaw else 0.0))
        rk.append(float(rk[-1] if rk else 0.0))
        s = np.append(s, spline.s[-1])

    return np.asarray(rx), np.asarray(ry), np.asarray(ryaw), np.asarray(rk), np.asarray(s)
