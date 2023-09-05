import math

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import signal, stats

"""
This scripts can be used to estimate parameters of a point mass dynamic model that represent a Carla vehicle. These parameters are used in the CEI model agent.
It uses data recorded in carla of a vehicle that fully accelerates and then fully brakes.  
"""


def get_rotation_matrix_from_carla(roll, pitch, yaw, degrees=True):
    """
    calculation based on this github issue: https://github.com/carla-simulator/carla/issues/58 because carla uses some rather unconventional conventions.
    """
    if degrees:
        roll, pitch, yaw = np.radians([roll, pitch, yaw])

    yaw_matrix = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])

    pitch_matrix = np.array([
        [math.cos(pitch), 0, -math.sin(pitch)],
        [0, 1, 0],
        [math.sin(pitch), 0, math.cos(pitch)]
    ])

    roll_matrix = np.array([
        [1, 0, 0],
        [0, math.cos(roll), math.sin(roll)],
        [0, -math.sin(roll), math.cos(roll)]
    ])

    rotation_matrix = yaw_matrix @ pitch_matrix @ roll_matrix
    return rotation_matrix


def convert_string_to_list(string):
    assert string[0] == '['
    assert string[-1] == ']'

    list_of_strings = string[1:-1].split(', ')
    list_of_floats = [float(s) for s in list_of_strings]
    return list_of_floats


def load_data(file_name):
    data = pd.read_csv(file_name, sep='; ')

    for c in data.columns:
        if data[c].dtype == object:
            data[c] = data[c].apply(convert_string_to_list)
    return data


def convert_to_vehicle_frame(transform, data):
    rotation_matrix = get_rotation_matrix_from_carla(transform[5], transform[4], transform[3])
    data_in_vehicle_frame = np.linalg.inv(rotation_matrix) @ data

    return data_in_vehicle_frame


if __name__ == '__main__':
    """
    This script is used to manually estimate parameters to represent the vehicle dynamics in CARLA. Two files where generated on a straight road:
    
    no_braking.csv is used to estimate the resistance when no throttle is applied. This includes the resistance of the engine and the resistance of air
    Both of these files are also used to determine the maximum (constant) acceleration when full throttle is applied --> l95: -1 to 0, 0.35's (engine damping) in npc vehicle: 0
    
    full_braking.csv is used to determine the minimal acceleration during braking. --> all back to how it was (l95): 0 to -1
    
    The found parameters are specified below.
    """
    PLOT = True

    resistance = [0.000203, 0.04799443, 0.00035266]
    max_acceleration = 4.460
    min_acceleration = -4.26

    data = load_data(file_name='full_braking.csv')

    carla_time_trace = (data['Carla Interface.time'] - data['Carla Interface.time'].iat[0]) * 1e-9
    npc_time_trace = (data['NPC Controller Manager.time'] - data['NPC Controller Manager.time'].iat[1]) * 1e-9
    npc_time_trace[0] = npc_time_trace[1]

    accelerations = np.array(data['Carla Interface.agents.NPC Vehicle_1.accelerations_in_world_frame'].to_list())
    velocities = data['Carla Interface.agents.NPC Vehicle_1.velocities_in_vehicle_frame']
    transform = np.array(data['Carla Interface.agents.NPC Vehicle_1.transform'].to_list())

    accelerations_in_vehicle_frame = data.apply(
        lambda r: convert_to_vehicle_frame(r['Carla Interface.agents.NPC Vehicle_1.transform'], r['Carla Interface.agents.NPC Vehicle_1.accelerations_in_world_frame']),
        axis=1)
    accelerations_in_vehicle_frame_np = np.array(accelerations_in_vehicle_frame.to_list())

    time_data = data.loc[(data['NPC Controller Manager.controllers.Full Throttle_1.throttle'] == 0.) &
                         (data['NPC Controller Manager.controllers.Full Throttle_1.brake'] == 0.), 'NPC Controller Manager.time']
    length = len(time_data)
    half = int(length / 2)
    try:
        initial_time_idle = time_data.unique()[50]  # half a second dead band to discard initial behavior
        final_time_idle = time_data.unique()[-1000]
    except IndexError:
        initial_time_idle = 0.
        final_time_idle = 0.

    time_data = data.loc[(data['NPC Controller Manager.controllers.Full Throttle_1.throttle'] == 1.), 'NPC Controller Manager.time']
    length = len(time_data)
    half = int(length / 2)

    initial_time_of_acceleration = time_data.unique()[300]  # 3 second dead band to discard initial behavior
    final_time_of_acceleration = time_data.unique()[-1]

    time_data = data.loc[(data['NPC Controller Manager.controllers.Full Throttle_1.brake'] == 1.), 'NPC Controller Manager.time']
    length = len(time_data)
    quarter = int(length / 4)

    try:
        initial_time_of_braking = time_data.iat[quarter]
        final_time_of_braking = time_data.iat[quarter + 60]
    except IndexError:
        initial_time_of_braking = 0
        final_time_of_braking = 0

    if final_time_idle and initial_time_idle:
        idle_mask = ((data['Carla Interface.time'] <= final_time_idle) &
                     (data['Carla Interface.time'] >= initial_time_idle))
        deceleration_during_idle = accelerations_in_vehicle_frame.loc[idle_mask]
        deceleration_during_idle = np.array(deceleration_during_idle.to_list())[:, 0]

        velocity_during_idle = velocities.loc[idle_mask]
        velocity_during_idle = np.array(velocity_during_idle.to_list())[:, 0]

        poly_result = np.polyfit(velocity_during_idle, deceleration_during_idle, deg=2)
        print(poly_result)

        resistance_estimate = [poly_result[0] * v ** 2 + poly_result[1] * v + poly_result[2] for v in velocity_during_idle]

        time_idle = carla_time_trace[idle_mask.to_numpy()]
        deceleration_due_to_air_resistance = resistance[0] * velocity_during_idle ** 2 + resistance[1] * velocity_during_idle + resistance[2]
        constant_deceleration_trace = deceleration_during_idle + deceleration_due_to_air_resistance

    acceleration_mask = ((data['Carla Interface.time'] <= final_time_of_acceleration) &
                         (data['Carla Interface.time'] >= initial_time_of_acceleration) &
                         (np.array(accelerations_in_vehicle_frame.to_list())[:, 0] > 1e-3))

    acceleration_phase = accelerations_in_vehicle_frame.loc[acceleration_mask]
    acceleration_phase = np.array(acceleration_phase.to_list())[:, 0]

    velocity_during_acceleration = velocities.loc[acceleration_mask]
    velocity_during_acceleration = np.array(velocity_during_acceleration.to_list())[:, 0]

    acceleration_time = carla_time_trace[acceleration_mask.to_numpy()]
    input_acceleration = acceleration_phase + resistance[0] * velocity_during_acceleration ** 2 + resistance[1] * velocity_during_acceleration + resistance[2]

    print('max_acceleration = %.3f' % (sum(input_acceleration) / len(input_acceleration)))

    if initial_time_of_braking and final_time_of_braking:
        brake_mask = ((data['Carla Interface.time'] <= final_time_of_braking) &
                      (data['Carla Interface.time'] >= initial_time_of_braking))
        braking_phase = accelerations_in_vehicle_frame.loc[brake_mask]
        braking_phase = np.array(braking_phase.to_list())[:, 0]

        velocity_during_braking = velocities.loc[brake_mask]
        velocity_during_braking = np.array(velocity_during_braking.to_list())[:, 0]

        braking_time = carla_time_trace[brake_mask.to_numpy()]
        input_braking = braking_phase + resistance[0] * velocity_during_braking ** 2 + resistance[1] * velocity_during_braking + resistance[2]
        min_acceleration = sum(input_braking) / len(input_braking)
        print('min_acceleration = %.3f' % min_acceleration)

    if PLOT:
        fig = plt.figure()
        plt.plot(carla_time_trace, accelerations[:, 0])
        plt.plot(carla_time_trace, accelerations[:, 1])
        plt.plot(carla_time_trace, accelerations[:, 2])
        plt.title('accelerations - world frame')

        fig = plt.figure()
        plt.plot(carla_time_trace, accelerations_in_vehicle_frame_np[:, 0])
        plt.plot(carla_time_trace, accelerations_in_vehicle_frame_np[:, 1])
        plt.plot(carla_time_trace, accelerations_in_vehicle_frame_np[:, 2])
        plt.title('accelerations - vehicle frame')

        velocities_for_plotting = np.array(velocities.to_list())
        fig = plt.figure()
        plt.plot(carla_time_trace, velocities_for_plotting[:, 0])
        plt.plot(carla_time_trace, velocities_for_plotting[:, 1])
        plt.plot(carla_time_trace, velocities_for_plotting[:, 2])
        plt.title('velocities - vehicle frame')

        fig = plt.figure()
        plt.plot(carla_time_trace, transform[:, 3], label='yaw')
        plt.plot(carla_time_trace, transform[:, 4], label='roll')
        plt.plot(carla_time_trace, transform[:, 5], label='pitch')
        plt.title('orientation')
        plt.legend()

        plt.figure()
        plt.plot(transform[:, 0], transform[:, 1])
        plt.title('position')

        plt.figure()
        plt.plot(npc_time_trace, data['NPC Controller Manager.controllers.Full Throttle_1.throttle'].to_numpy(), label='throttle controller')
        plt.plot(npc_time_trace, data['NPC Controller Manager.controllers.Full Throttle_1.brake'].to_numpy(), label='brake controller')
        plt.legend()
        plt.title('throttle')

        plt.figure()
        plt.plot(acceleration_time, input_acceleration, label='input acceleration')
        plt.plot(acceleration_time, acceleration_phase, label='net acceleration')
        if initial_time_idle and final_time_idle:
            plt.plot(time_idle, deceleration_during_idle, label='idle')
            plt.plot(time_idle, constant_deceleration_trace, label='should be constant')

        if initial_time_of_braking and final_time_of_braking:
            plt.plot(braking_time, input_braking, label='braking')
        plt.title('Acceleration Only')
        plt.legend()

        if final_time_idle and initial_time_idle:
            plt.figure()
            plt.plot(velocity_during_idle, deceleration_during_idle)
            plt.plot(velocity_during_idle, resistance_estimate)
            plt.title('deceleration over velocity')
            plt.xlabel('velocity [m/s]')
            plt.ylabel('acceleration [m/s^2]')

        plt.figure()
        plt.plot(velocity_during_acceleration, input_acceleration, label='input')
        plt.plot(velocity_during_acceleration, acceleration_phase, label='net acceleration')
        plt.title('acceleration over velocity')
        plt.xlabel('velocity [m/s]')
        plt.ylabel('acceleration [m/s^2]')
        plt.legend()

        if initial_time_of_braking and final_time_of_braking:
            plt.figure()
            plt.plot(velocity_during_braking, input_braking, label='input')
            plt.plot(velocity_during_braking, braking_phase, label='net acceleration')
            plt.title('deceleration over velocity')
            plt.xlabel('velocity [m/s]')
            plt.ylabel('acceleration [m/s^2]')
            plt.legend()

        plt.show()
