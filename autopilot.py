import time

import carla
import pandas as pd
import class_av
from plotting_scene import pltng_scene
from get_vehicles import gt_vhcl


# def autopilot():

client = carla.Client('131.180.28.182', 2000)  # Use the correct IP address and port
client.set_timeout(10.0)  # Set a timeout value
# Retrieve the list of vehicles
carla_world = client.get_world()
carla_map = carla_world.get_map()
nested_car_carla, human_car_carla, fede_car_carla = gt_vhcl(carla_world, carla_map)
traffic_manager = client.get_trafficmanager(8000)
traffic_manager.global_percentage_speed_difference(-103)
running = True
autopilot_flag = True
while running:
    print("AV veloity: ", nested_car_carla.get_velocity())
    print("HV veloity: ", human_car_carla.get_velocity())
    print("Velocity error: ", nested_car_carla.get_velocity() - nested_car_carla.get_velocity())
    if nested_car_carla.get_location().x == 0:
        print("Cars were destroyed, waiting for new condition...")
        running = False
    if fede_car_carla.get_velocity() > 0 and autopilot_flag is True:
        nested_car_carla.vehicle.set_autopilot(True)
        time.sleep(0.155)
        human_car_carla.vehicle.set_autopilot(True)
        autopilot_flag = False
    if nested_car_carla.get_location().x >= 360:
        nested_car_carla.vehicle.set_autopilot(False)
        human_car_carla.vehicle.set_autopilot(False)
        running = False
        print('delta_x: ', nested_car_carla.get_location().x - human_car_carla.get_location().x)
        print('delta_y: ', nested_car_carla.get_location().y - human_car_carla.get_location().y)
        print('delta_v: ', nested_car_carla.get_velocity() - human_car_carla.get_velocity())
