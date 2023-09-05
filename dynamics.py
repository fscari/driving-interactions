import numpy as np
import theano as th
import theano.tensor as tt
from scipy import interpolate


class Dynamics(object):
    def __init__(self, nx, nu, f, dt=None):
        self.nx = nx
        self.nu = nu
        self.dt = dt
        if dt is None:
            self.f = f
        else:
            self.f = lambda x, u: x + dt * f(x, u)

    def __call__(self, x, u):
        return self.f(x, u)


# class CarDynamics(Dynamics):
#     ## old Sadigh'smodel
#     # def __init__(self, dt=0.1, ub=[(-3., 3.), (-1., 1.)], friction=1.):
#     #     def f(x, u):
#     #         return tt.stacklists([
#     #             x[3] * tt.cos(x[2]),
#     #             x[3] * tt.sin(x[2]),
#     #             x[3] * u[0],
#     #             u[1] - x[3] * friction
#     #         ])
#     #
#     #     Dynamics.__init__(self, 4, 2, f, dt)
#
#     ## new model
#     def __init__(self, dt=0.1, ub=[(-3., 3.), (-1., 1.)], friction=1.):
#         # Linearized Longitudinal Dynamics
#         resistance = [0.000203, 0.04799443, 0.00035266]
#
#         def longitudinal_dynamics(v):
#             # Linearized resistance model
#             res = resistance[0] * v ** 2 + resistance[1] * v + resistance[2]
#             return -res
#
#         def f(x, u):
#             # x[0] - x position
#             # x[1] - y position
#             # x[2] - yaw angle
#             # x[3] - velocity
#
#             # u[0] - steering angle
#             # u[1] - acceleration command
#
#             # Linearized longitudinal acceleration
#             a_long = u[1] + longitudinal_dynamics(x[3])
#
#             # Bicycle Model for lateral motion and yaw rate
#             dx = x[3] * tt.cos(x[2])
#             dy = x[3] * tt.sin(x[2])
#             dpsi = x[3] * u[0]
#
#             return tt.stacklists([dx, dy, dpsi, a_long])
#         Dynamics.__init__(self, 4, 2, f, dt)

#new new model
class CarDynamics(Dynamics):
    def __init__(self, dt=0.1, Cf=1200, Cr=1200, m=1500, Iz=2000, a=1.4, b=1.6, ub=[(-3., 3.), (-1., 1.)], friction=1.):
        # Linearized Longitudinal Dynamics
        resistance = [0.000203, 0.04799443, 0.00035266]
        max_acceleration = 4.460
        min_acceleration = -4.26
        # Cf and Cr should be C*2*pi/180?
        self.Cf = 17  # front tire cornering stiffness
        self.Cr = 17  # rear tire cornering stiffness
        self.m = 1500.00  # mass of the vehicle
        self.Iz = 1/12*self.m*((2.39*2)**2+(1.03*2)**2)  # yaw moment of inertia
        self.a = 2.38  # distance from CG to front axle
        self.b = 2.38  # distance from CG to rear axle

        steering_curve_points = [(0, 1), (20, 0.9), (60, 0.8), (120, 0.7)]
        self.steering_curve_points_deg = [point[0] for point in steering_curve_points]
        self.steering_curve_effect = [point[1] for point in steering_curve_points]

        def steering_interpolator(steering_angle_deg):
            return np.interp(steering_angle_deg, self.steering_curve_points_deg, self.steering_curve_effect)

        def longitudinal_dynamics(v):
            # Linearized resistance model
            resistance_force = resistance[0] * v ** 2 + resistance[1] * v + resistance[2]
            # max_accel = max_acceleration - resistance_force / self.m
            # min_accel = min_acceleration - resistance_force / self.m
            return resistance_force
        def f(x, u):
            # x[0] - x position
            # x[1] - y position
            # x[2] - yaw angle
            # x[3] - forward velocity (v)
            # x[4] - lateral velocity (v_y)
            # x[5] - yaw rate (r)

            # u[0] - steering angle
            # u[1] - acceleration command

            # Linearized longitudinal acceleration
            a_long = u[1] + longitudinal_dynamics(x[3])
            tt.clip(a_long, min_acceleration, max_acceleration)
            # Slip angle (for small angles)
            beta = tt.arctan2(x[4], x[3])

            # Bicycle Model Dynamics
            dx = x[3] * tt.cos(x[2] + beta)
            dy = x[3] * tt.sin(x[2] + beta)
            dpsi = x[5]
            dv = a_long
            dv_y = -x[3] * x[5] + (1 / self.m) * (self.Cf * u[0] + self.Cr * beta)

            # Calculate the interpolated effect of the steering curve on yaw rate
            steering_angle_deg = np.rad2deg(u.get_value()[0])
            steering_effect = steering_interpolator(steering_angle_deg)

            # Modify yaw rate calculation with steering curve effect
            # dr = (1 / self.Iz) * (self.a * self.Cf * u[0] - self.b * self.Cr * beta) * steering_effect

            dr = (1 / self.Iz) * (self.a * self.Cf * u[0] - self.b * self.Cr * beta)

            return tt.stacklists([dx, dy, dpsi, dv, dv_y, dr])

        Dynamics.__init__(self, 6, 2, f, dt)




if __name__ == '__main__':
    dyn = CarDynamics(0.1)
    x = tt.vector()
    u = tt.vector()
    dyn(x, u)
