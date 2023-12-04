import carla
import time
import copy
import sys
import pandas as pd
from controller_init_vehicle1 import cntrlr_init
from plotting_scene import pltng_scene
from get_vehicles import gt_vhcl
from get_vehicles_M import gt_vhcl_M


def main(n1, n2):
    sys.setrecursionlimit(5000)

    experiment_nr = n1
    session = n2

    number_of_iterations = 10
    vehicle_nr = 1
    # Connect to the CARLA server
    client = carla.Client('131.180.29.232', 2000)  # Use the correct IP address and port
    client.set_timeout(10.0)  # Set a timeout value
    carla_world = client.get_world()
    carla_map = carla_world.get_map()
    traffic_manager = client.get_trafficmanager(8000)
    traffic_manager.global_percentage_speed_difference(-103)
    # nested_car_carla, human_car_carla, fede_car_carla = gt_vhcl(carla_world, carla_map, vehicle_nr)

    # Saddigh
    dt = 0.01
    # theta = [lanes, fances, road, speed, trajectoy.h]
    # theta = [30, -15, 75, 1, -100] # final? used in pilot 1
    theta = [30, -10, 75, 1, -50]  # test?
    T = 10
    theta.append(T)
    humancar_right, nestedcar_left, humancar_left, nestedcar_right = cntrlr_init(dt, theta, T)
    # Initialisation
    steering = 0
    throttle = 0
    count_left = 0
    count_right = 0
    nestedcar_left.control(0.0, 0.0)
    nestedcar_right.control(0.0, 0.0)

    # Initialize lists to hold the copies
    nestedcar_left_copies = []
    nestedcar_right_copies = []
    humancar_left_copies = []
    humancar_right_copies = []

    nestedcar_left_copies.append(nestedcar_left)
    nestedcar_right_copies.append(nestedcar_right)
    humancar_left_copies.append(humancar_left)
    humancar_right_copies.append(humancar_right)
    # Creating 10 copies of each initialized object
    for i in range(4):
        # For nestedcar_left
        temp_nestedcar_left = copy.copy(nestedcar_left)
        temp_nestedcar_left.optimizer = nestedcar_left.optimizer.customcopy()
        nestedcar_left_copies.append(temp_nestedcar_left)

        # For nestedcar_right
        temp_nestedcar_right = copy.copy(nestedcar_right)
        temp_nestedcar_right.optimizer = nestedcar_right.optimizer.customcopy()
        nestedcar_right_copies.append(temp_nestedcar_right)

        # For humancar_left
        temp_humancar_left = copy.deepcopy(humancar_left)
        humancar_left_copies.append(temp_humancar_left)

        # For humancar_right
        temp_humancar_right = copy.deepcopy(humancar_right)
        humancar_right_copies.append(temp_humancar_right)

        print("-----------------------")
        print(i+1)
        print("-----------------------")

    # I'll always start with the AV on the left
    nested_car = nestedcar_left_copies[0]
    human_car = humancar_right_copies[0]
    condition_name = 'AV left'
    condition_id = 0
    count_left += 1
    # count = count_left

    start_time = time.time()
    done = False
    first = True
    autopilot_flag = False
    count = 0
    for i in range(number_of_iterations):
        vehicles_data = pd.DataFrame(columns=['times', 'x_positions_human_car', 'y_positions_human_car',
                                              'x_positions_nested_car', 'y_positions_nested_car', 'steering_input'])
        # input("Press Enter to initialise the experiment: ")
        if first is not True:
            first = False
            if i == 3 or i == 5 or i ==7 or i == 8:
                nested_car = nestedcar_left_copies[count_left]
                human_car = humancar_right_copies[count_left]
                condition_name = 'AV left'
                condition_id = 0
                count_left += 1
                count = count_left
                steering = 0.0
                throttle = 0
            elif i == 1 or i == 2 or i == 4 or i == 6 or i == 9:
                nested_car = nestedcar_right_copies[count_right]
                human_car = humancar_left_copies[count_right]
                condition_name = 'AV right'
                condition_id = 1
                count_right += 1
                count = count_right
                steering = 0.0
                throttle = 0
        running = True
        start = False
        brake = 0
        input("Press Enter to load the experiment:")
        if session == 1:
            if i == 2 or i == 5 or i == 6 or i == 8 or i == 9:
                nested_car_carla, human_car_carla, fede_car_carla = gt_vhcl_M(carla_world, carla_map, vehicle_nr)
            else:
                nested_car_carla, human_car_carla, fede_car_carla = gt_vhcl(carla_world, carla_map, vehicle_nr)
        elif session == 2:
            if i == 0 or i == 1 or i == 3 or i == 4 or i == 7:
                nested_car_carla, human_car_carla, fede_car_carla = gt_vhcl_M(carla_world, carla_map, vehicle_nr)
            else:
                nested_car_carla, human_car_carla, fede_car_carla = gt_vhcl(carla_world, carla_map, vehicle_nr)
        # input("Press Enter to start the experiment:")
        while running:
            if nested_car_carla.get_location().x == 0:
                print("Cars were destroyed, waiting for new condition...")
                running = False
            if fede_car_carla.get_velocity() > 0 or start is True:
                if start is False:
                    start_time = time.time()
                    loop_time = start_time
                    nested_car_carla.vehicle.set_autopilot(True)
                    # time.sleep(0.17)
                    human_car_carla.vehicle.set_autopilot(True)
                    autopilot_flag = True
                    start = True
                if nested_car_carla.get_location().x >= 359 and nested_car_carla.get_location().x < 560:
                    if autopilot_flag:
                        nested_car_carla.vehicle.set_autopilot(False)
                        human_car_carla.vehicle.set_autopilot(False)
                        autopilot_flag = False
                    loop_time = time.time()
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
                    control = carla.VehicleControl(throttle=throttle, steer=steering, brake=brake, hand_brake=False)
                    nested_car_carla.vehicle.apply_control(control)
                    carla_world.tick()

                    # Apply the control commands to the vehicle in Controller
                    nested_car.move(nested_car_carla, human_car_carla)
                    human_car.move(human_car_carla)

                    sleep_time = dt - (time.time() - loop_time)
                    # if sleep_time > 0:
                    #     time.sleep(sleep_time)
                    # nested_car.control(steering, throttle)
                elif nested_car_carla.get_location().x >= 560:
                    nested_car_carla.vehicle.set_autopilot(True)

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
        last_condition_name = condition_name
        first = False
        pltng_scene(vehicles_data, theta, session, vehicle_nr, condition_name, experiment_nr, count)
    print("Experiment finished!")

if __name__ == '__main__':
    n1, n2 = sys.argv[1:]
    n1 = int(n1)
    n2 = int(n2)
    main(n1, n2)
