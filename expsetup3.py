import carla
import time
import pandas as pd
from controller_init import cntrlr_init
from plotting_scene import pltng_scene
from get_vehicles import gt_vhcl
import class_av
from run_av import run_av


experiment_nr = 1
number_of_iterations = 5
title = "Experiment Nr: " + str(experiment_nr) + "; Iteration Nr: "

target_speed = 16.67  # m/s
look_ahead_dist = 15
# Connect to the CARLA server
client = carla.Client('131.180.28.182', 2000)  # Use the correct IP address and port
client.set_timeout(10.0)  # Set a timeout value
# Retrieve the list of vehicles
carla_world = client.get_world()
carla_map = carla_world.get_map()
nested_car_carla, human_car_carla = gt_vhcl(carla_world, carla_map)
if nested_car_carla.get_location().y > 0:
    controller = class_av.AV(carla_map, nested_car_carla, target_speed, look_ahead_dist)
else:
    controller = None
# Saddigh
dt = 0.01
# theta = [lanes, fances, road, speed, trajectoy.h]
theta =  [10, -46.271, 90.15, 8.531, -100.604]
# theta = [5, -46.271, 90.15, 8.531, -100.604] # works well with HR
# theta = [100., -500., 10., 10., -60.]
T = 10
theta.append(T)
human_car, nested_car = cntrlr_init(dt, human_car_carla, nested_car_carla, theta, T)
nested_car_origin = nested_car

hand_brake = False
running = True

print("theta: ")
print(theta)

steering = 0
throttle = 0
start_time = time.time()
nested_car_origin.control(0.001685006396878057, 0.5)
done = False
for i in range(number_of_iterations):
    times = []
    vehicles_data = pd.DataFrame(columns=['times', 'x_positions_human_car', 'y_positions_human_car', 'x_positions_nested_car',
                               'y_positions_nested_car', 'steering_input'])
    input("Press Enter to start the experiment:")
    if done:
        nested_car_carla, human_car_carla = gt_vhcl(carla_world, carla_map)
        if nested_car_carla.get_location().y > 0:
            controller = class_av.AV(carla_map, nested_car_carla, target_speed, look_ahead_dist)
    running = True
    first = True
    start = False
    steering = 0
    throttle = 0
    brake = 0

    while running:
        if nested_car_carla.get_location().x == 0:
            print("Cars were destroyed, waiting for new condition...")
            done = True
            running = False
        if human_car_carla.get_velocities().x > 0 and human_car_carla.get_location().x >= 50:
            if start is False:
                start_time = time.time()
                loop_time = start_time
                start = True
            if nested_car_carla.get_location().x >= 365:
                loop_time = time.time()
                nested_car.control(steering, throttle)
                print("------------------------------")
                if first:
                    first = False
                    nested_car.optimizer = nested_car_origin.optimizer
                else:
                    nested_car.control(steering, throttle)
                u = nested_car.traj.u[0].get_value()
                # Set control commands
                steering = u[0]
                throttle = u[1]
                if throttle < 0:
                    brake = abs(throttle)
                    throttle = 0
                else:
                    brake = 0
                # Apply the control commands to the vehicle in Carla
                control = carla.VehicleControl(throttle=throttle, steer=steering, brake=brake, hand_brake=hand_brake)
                nested_car_carla.vehicle.apply_control(control)
                carla_world.tick()

                # Apply the control commands to the vehicle in Controller
                nested_car.move(nested_car_carla, human_car_carla)
                human_car.move(human_car_carla)

                print("input: ")
                print(steering, throttle)
                print("state: ")
                print(nested_car_carla.get_location().x, nested_car_carla.get_location().y,
                      nested_car_carla.get_rotation().yaw, nested_car_carla.get_velocity())

                sleep_time = dt - (time.time() - loop_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
            elif nested_car_carla.get_location().x <= 365:
                if controller is not None:
                    vehicles_data = run_av(carla_world, human_car_carla, nested_car_carla, controller, vehicles_data)

            new_data = {
                'times': carla_world.get_snapshot().timestamp.elapsed_seconds,
                'x_positions_human_car': human_car_carla.get_location().x,
                'y_positions_human_car': human_car_carla.get_location().y,
                'x_positions_nested_car': nested_car_carla.get_location().x,
                'y_positions_nested_car': nested_car_carla.get_location().y,
                'steering_input': steering
            }
            vehicles_data = pd.concat([vehicles_data, pd.DataFrame([new_data])], ignore_index=True)
    end_time = time.time()
    controller = False
    title = title + str(i)
    pltng_scene(vehicles_data, theta, title)
    title = title[:-1]
print("Experiment finished!")
