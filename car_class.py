import numpy as np
import utils
from trajectory import Trajectory
import math

class Car(object):
    def __init__(self, dyn, x0, color='yellow', T=10):
        self.data0 = {'x0': x0}
        self.bounds = [(-1., 1.), (-1., 1.)]
        self.T = T
        self.dyn = dyn
        self.traj = Trajectory(T, dyn)
        self.traj.x0.set_value(x0)
        self.linear = Trajectory(T, dyn)
        self.linear.x0.set_value(x0)
        self.color = color
        self.default_u = np.zeros(self.dyn.nu)

    def reset(self):
        self.traj.x0.set_value(self.data0['x0'])
        self.linear.x0.set_value(self.data0['x0'])
        for t in range(self.T):
            self.traj.u[t].set_value(np.zeros(self.dyn.nu))
            self.linear.u[t].set_value(self.default_u)

    def move(self, carla_vehicle):
        new_state = [carla_vehicle.get_location().x, carla_vehicle.get_location().y,
                     math.radians(carla_vehicle.get_rotation().yaw), carla_vehicle.get_velocity(),
                     carla_vehicle.get_velocities().y, carla_vehicle.get_angular_velocities().z]
        self.traj.tick(new_state)
        self.traj.x0.set_value(self.x)
        self.linear.x0.set_value(self.traj.x0.get_value())

    @property
    def x(self):
        return self.traj.x0.get_value()

    @property
    def u(self):
        return self.traj.u[0].get_value()

    @u.setter
    def u(self, value):
        self.traj.u[0].set_value(value)

    def control(self, steer, gas):
        pass


class UserControlledCar(Car):
    def __init__(self, *args, **vargs):
        Car.__init__(self, *args, **vargs)
        self.bounds = [(-1., 1.), (-1., 1.)]
        self.follow = None
        self.fixed_control = None
        self._fixed_control = None

    def fix_control(self, ctrl):
        self.fixed_control = ctrl
        self._fixed_control = ctrl

    def control(self, steer, gas):
        if self.fixed_control is not None:
            self.u = self.fixed_control[0]
            print(self.fixed_control[0])
            if len(self.fixed_control) > 1:
                self.fixed_control = self.fixed_control[1:]
        elif self.follow is None:
            self.u = [steer, gas]
        else:
            u = self.follow.u[0].get_value()
            if u[1] >= 1.:
                u[1] = 1.
            if u[1] <= -1.:
                u[1] = -1.
            self.u = u

    def reset(self):
        Car.reset(self)
        self.fixed_control = self._fixed_control


class NestedOptimizerCar(Car):
    def __init__(self, *args, **vargs):
        Car.__init__(self, *args, **vargs)
        self.bounds = [(-math.pi, math.pi), (-1, 1)]

    @property
    def human(self):
        return self._human

    @human.setter
    def human(self, value):
        self._human = value
        self.traj_h = Trajectory(self.T, self.human.dyn)

    def move(self, nested_car_carla, human_car):
        Car.move(self, nested_car_carla)
        new_state = [human_car.get_location().x, human_car.get_location().y,
                     math.radians(human_car.get_rotation().yaw), human_car.get_velocity(),
                     human_car.get_velocities().y, human_car.get_angular_velocities().z]
        self.traj_h.tick(new_state)

    @property
    def rewards(self):
        return self._rewards

    @rewards.setter
    def rewards(self, vals):
        self._rewards = vals
        self.optimizer = None

    def control(self, steer, gas):
        if self.optimizer is None:
            reward_h, reward_r = self.rewards
            reward_h = self.traj_h.reward(reward_h)
            reward_r = self.traj.reward(reward_r)
            self.optimizer = utils.NestedMaximizer(reward_h, self.traj_h.u, reward_r, self.traj.u)
        self.traj_h.x0.set_value(self.human.x)
        self.optimizer.maximize(bounds=self.bounds)
        return self.u