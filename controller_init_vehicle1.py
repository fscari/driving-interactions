import dynamics, lane, car_class, feature, world
import math


def cntrlr_init(dt, human_car_carla, nested_car_carla, theta_set, T):
    dyn = dynamics.CarDynamics(dt)
    humancar_right = car_class.UserControlledCar(dyn, [360, 1.75, 0.0, 16.67, 0.0, 0.0], color='yellow', T=T)
    nestedcar_left = car_class.NestedOptimizerCar(dyn, [360, -1.75, 0.0, 16.67, 0.0, 0.0], color='blue', T=T)
    humancar_left = car_class.UserControlledCar(dyn, [360, -1.75, 0.0, 16.67, 0.0, 0.0], color='yellow', T=T)
    nestedcar_right = car_class.NestedOptimizerCar(dyn, [360, 2.00, -1.43, 16.67, -0.40, 0.83], color='blue', T=T)
    # nestedcar_right = car_class.NestedOptimizerCar(dyn, [363, 1.5, -2, 16.67, -0.2,
    #                                                0.5], color='blue', T=T)
    # y = 2.2786035537719727

    # Condition 0 Human right AV left
    experiment_env_0 = world.World()
    experiment_env_0.cars.append(humancar_right)
    experiment_env_0.cars.append(nestedcar_left)
    experiment_env_0.cars[1].human = experiment_env_0.cars[0]
    clane1 = lane.StraightLane([350, -1.75], [800, -1.75], 3.5)
    clane2 = lane.StraightLane([350, 1.75], [550, 1.75], 3.5)
    fakelane = lane.StraightLane([650, 1.75], [800, 1.75], 3.5)
    experiment_env_0.lanes += [clane1, clane2]
    experiment_env_0.roads += [clane1]
    # experiment_env.fences += [clane1.shifted(-1), clane2.shifted(1), fakelane.shifted(0.5)] # works fine
    experiment_env_0.fences += [clane1.shifted(-0.715), clane2.shifted(0.715), fakelane.shifted(0.2)]
    theta = theta_set
    r_h = experiment_env_0.simple_reward([nestedcar_left.traj], theta) + 100. * feature.bounded_control(humancar_right.bounds)
    r_r = experiment_env_0.simple_reward(nestedcar_left, theta, speed=16.67)
    experiment_env_0.cars[1].rewards = (r_h, r_r)


    # Condition 1 Human left AV right
    experiment_env_1 = world.World()
    experiment_env_1.cars.append(humancar_left)
    experiment_env_1.cars.append(nestedcar_right)
    experiment_env_1.cars[1].human = experiment_env_1.cars[0]
    clane1 = lane.StraightLane([360, -1.75], [800, -1.75], 3.5)
    clane2 = lane.StraightLane([360, 1.75], [550, 1.75], 3.5)
    fakelane = lane.StraightLane([650, 1.75], [800, 1.75], 3.5)
    experiment_env_1.lanes += [clane1, clane2]
    experiment_env_1.roads += [clane1]
    # experiment_env.fences += [clane1.shifted(-1), clane2.shifted(1), fakelane.shifted(0.5)] # works fine
    experiment_env_1.fences += [clane1.shifted(-0.715), clane2.shifted(0.715), fakelane.shifted(0.2)]
    theta = theta_set
    r_h = experiment_env_1.simple_reward([nestedcar_right.traj], theta) + 100. * feature.bounded_control(humancar_left.bounds)
    r_r = experiment_env_1.simple_reward(nestedcar_right, theta, speed=16.67)
    experiment_env_1.cars[1].rewards = (r_h, r_r)

    return humancar_right, nestedcar_left, humancar_left, nestedcar_right
