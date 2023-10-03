import dynamics, lane, car_class, feature, world
import math


def cntrlr_init(dt, human_car_carla, nested_car_carla, theta_set, T):
    dyn = dynamics.CarDynamics(dt)
    humancar = car_class.UserControlledCar(dyn, [human_car_carla.get_location().x, human_car_carla.get_location().y,
                                           math.radians(human_car_carla.get_rotation().yaw), human_car_carla.get_velocities().x,
                                           human_car_carla.get_velocities().y, human_car_carla.get_angular_velocities().z],
                                     color='yellow', T=T)
    nestedcar = car_class.NestedOptimizerCar(dyn, [nested_car_carla.get_location().x, nested_car_carla.get_location().y,
                                             math.radians(nested_car_carla.get_rotation().yaw), nested_car_carla.get_velocities().x,
                                             nested_car_carla.get_velocities().y, nested_car_carla.get_angular_velocities().z],
                                       color='blue', T=T)
    experiment_env = world.World()
    experiment_env.cars.append(humancar)
    experiment_env.cars.append(nestedcar)
    experiment_env.cars[1].human = experiment_env.cars[0]
    clane1 = lane.StraightLane([367, -1.75], [800, -1.75], 3.5)
    clane2 = lane.StraightLane([367, 1.75], [550, 1.75], 3.5)
    fakelane = lane.StraightLane([600, 1.75], [800, 1.75], 3.5)
    experiment_env.lanes += [clane1, clane2]
    experiment_env.roads += [clane1]
    experiment_env.fences += [clane1.shifted(-1), clane2.shifted(1), fakelane.shifted(0.5)]
    theta = theta_set
    r_h = experiment_env.simple_reward([nestedcar.traj], theta) + 100. * feature.bounded_control(humancar.bounds)
    r_r = experiment_env.simple_reward(nestedcar, theta, speed=22)
    experiment_env.cars[1].rewards = (r_h, r_r)

    return humancar, nestedcar
