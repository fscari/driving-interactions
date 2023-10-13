import carla
import time
import theano as th
import pandas as pd
from controller_init import cntrlr_init
from plotting_scene import pltng_scene
from get_vehicles import gt_vhcl
import class_av
from run_av import run_av

# th.config.optimizer = 'fast_compile'
# th.config.mode = 'FAST_COMPILE'

experiment_nr = 0
try:
    experiment_nr = int(input("Please enter the Experiment NR. (integer): "))
    print(f"You entered: {experiment_nr}")
except ValueError:
    print("That's not a valid integer!")

number_of_iterations = 4

# Connect to the CARLA server
client = carla.Client('131.180.28.182', 2000)  # Use the correct IP address and port
client.set_timeout(10.0)  # Set a timeout value
carla_world = client.get_world()
carla_map = carla_world.get_map()
traffic_manager = client.get_trafficmanager(8000)
traffic_manager.global_percentage_speed_difference(-103)
nested_car_carla, human_car_carla, fede_car_carla = gt_vhcl(carla_world, carla_map)

# Saddigh
dt = 0.01
# theta = [lanes, fances, road, speed, trajectoy.h]
theta =  [10, -5, 80, 5, -100] # might be final one?
T = 10
theta.append(T)
humancar_right, nestedcar_left, humancar_left, nestedcar_right = cntrlr_init(dt, human_car_carla, nested_car_carla,
                                                                             theta, T)
# print("theta: ", theta)
steering = 0
throttle = 0
start_time = time.time()
nestedcar_left.control(0.0, 0.0)
nestedcar_right.control(0.0, 0.0)
done = False
first = True
autopilot_flag = False
count_left = 0
count_right = 0
count = 0
for i in range(number_of_iterations):
    vehicles_data = pd.DataFrame(columns=['times', 'x_positions_human_car', 'y_positions_human_car',
                                          'x_positions_nested_car', 'y_positions_nested_car', 'steering_input'])
    input("Press Enter to start the experiment:")
    nested_car_carla, human_car_carla, fede_car_carla = gt_vhcl(carla_world, carla_map)
    if nested_car_carla.get_location().y < 0:
        nested_car = nestedcar_left
        human_car = humancar_right
        condition_name = 'AV left'
        condition_id = 0
        count_left += 1
        count = count_left
        steering = 0.0
        throttle = 0
    else:
        nested_car = nestedcar_right
        human_car = humancar_left
        condition_name = 'AV right'
        condition_id = 1
        count_right += 1
        count = count_right
        steering = 0.0
        throttle = 0
    running = True
    start = False
    brake = 0

    while running:
        if nested_car_carla.get_location().x == 0:
            print("Cars were destroyed, waiting for new condition...")
            running = False
        if fede_car_carla.get_velocity() > 0 or start is True:
            if start is False:
                start_time = time.time()
                loop_time = start_time
                nested_car_carla.vehicle.set_autopilot(True)
                human_car_carla.vehicle.set_autopilot(True)
                autopilot_flag = True
                start = True
            if human_car_carla.get_location().x >= 360:
                if autopilot_flag:
                    nested_car_carla.vehicle.set_autopilot(False)
                    # time.sleep(0.155)
                    human_car_carla.vehicle.set_autopilot(False)
                    autopilot_flag = False
                loop_time = time.time()
                nested_car.control(steering, throttle)
                print("------------------------------")
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
                control = carla.VehicleControl(throttle=throttle, steer=steering, brake=brake, hand_brake=False)
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
                # if sleep_time > 0:
                #     time.sleep(sleep_time)
                # nested_car.control(steering, throttle)
            new_data = {
                'times': carla_world.get_snapshot().timestamp.elapsed_seconds,
                'x_positions_human_car': human_car_carla.get_location().x,
                'y_positions_human_car': human_car_carla.get_location().y,
                'x_positions_nested_car': nested_car_carla.get_location().x,
                'y_positions_nested_car': nested_car_carla.get_location().y,
                'steering_input': steering
            }
            vehicles_data = pd.concat([vehicles_data, pd.DataFrame([new_data])], ignore_index=True)
            if nested_car_carla.get_location().x and human_car_carla.get_location().x >= 350:
                # Apply the control commands to the vehicle in Controller
                nested_car.move(nested_car_carla, human_car_carla)
                human_car.move(human_car_carla)
    end_time = time.time()
    nested_car = None
    human_car = None
    pltng_scene(vehicles_data, theta, condition_name, experiment_nr, count)
print("Experiment finished!")
