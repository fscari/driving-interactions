import lane
import car
import math
import feature
import dynamics
import visualize
import theano as th
import numpy as np


th.config.optimizer_verbose = True
th.config.allow_gc = False
th.config.optimizer = 'fast_compile'


class Object(object):
    def __init__(self, name, x):
        self.name = name
        self.x = np.asarray(x)


class World(object):
    def __init__(self):
        self.cars = []
        self.lanes = []
        self.roads = []
        self.fences = []
        self.objects = []


    def simple_reward(self, trajecory, weights, trajs=None, lanes=None, roads=None, fences=None, speed=1., speed_import=1.):
        if lanes is None:
            lanes = self.lanes
        if roads is None:
            roads = self.roads
        if fences is None:
            fences = self.fences
        if trajs is None:
            trajs = [c.linear for c in self.cars]
        elif isinstance(trajs, car.Car):
            trajs = [c.linear for c in self.cars if c != trajs]
        r = 0.1 * feature.control()
        # theta = [1., -50., 10., 10., -60.]
        theta = weights # my trials
        # theta = [.959, -46.271, 9.015, 8.531, -57.604]
        for lane in lanes:
            r = r + theta[0] * lane.gaussian()
        for fence in fences:
            r = r + theta[1] * fence.gaussian()
        for road in roads:
            r = r + theta[2] * road.gaussian(10.)
        if speed is not None:
            r = r + speed_import * theta[3] * feature.speed(speed)
        for traj in trajs:
            r = r + theta[4] * traj.gaussian()
        return r

def world0():
    dyn = dynamics.CarDynamics(0.1)
    world = World()
    clane = lane.StraightLane([0., -1.], [0., 1.], 0.13)
    world.lanes += [clane, clane.shifted(1), clane.shifted(-1)]
    world.roads += [clane]
    world.fences += [clane.shifted(2), clane.shifted(-2)]
    # world.cars.append(car.UserControlledCar(dyn, [-0.13, 0., math.pi / 2., 0.3], color='red'))
    # world.cars.append(car.NestedOptimizerCar(dyn, [0.0, 0.5, math.pi / 2., 0.3], color='yellow'))
    # world.cars[1].human = world.cars[0]
    # r_h = world.simple_reward([world.cars[1].traj]) + 100. * feature.bounded_control(world.cars[0].bounds)

    @feature.feature
    def human_speed(t, x, u):
        return -world.cars[1].traj_h.x[t][3] ** 2

    # r_r = world.simple_reward(world.cars[1], speed=0.5)
    # world.cars[1].rewards = (r_h, r_r)
    return world

if __name__ == '__main__':
    world = playground()
    # world.cars = world.cars[:0]
    vis = visualize.Visualizer(0.1, magnify=1.2)
    vis.main_car = None
    vis.use_world(world)
    vis.paused = True


    @feature.feature
    def zero(t, x, u):
        return 0.


    r = zero
    # for lane in world.lanes:
    #    r = r+lane.gaussian()
    # for fence in world.fences:
    #    r = r-3.*fence.gaussian()
    r = r - world.cars[0].linear.gaussian()
    # vis.visible_cars = [world.cars[0]]
    vis.set_heat(r)
    vis.run()
