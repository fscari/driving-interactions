import carla
import time
from countdown import cntdwn
from controller_init import cntrlr_init
from plotting_scene import pltng_scene
from get_vehicles import gt_vhcl
# from cruise_control import cruise_control


experiment_nr = 1
number_of_iterations = 2
title = "Experiment Nr: " + str(experiment_nr) + "; Iteration Nr: "

# Connect to the CARLA server
client = carla.Client('131.180.28.182', 2000)  # Use the correct IP address and port
client.set_timeout(10.0)  # Set a timeout value
# Retrieve the list of vehicles
carla_world = client.get_world()
carla_map = carla_world.get_map()
nested_car_carla, human_car_carla = gt_vhcl(carla_world, carla_map)

# Saddigh
dt = 0.01
# theta = [lanes, fances, road, speed, trajectoy.h]
# theta = [3, -60, 90, 8.5, -100.5]
theta = [5, -46.271, 90.15, 8.531, -100.604] # works well with HR
T = 10
theta.append(T)
human_car, nested_car = cntrlr_init(dt, human_car_carla, nested_car_carla, theta, T)
nested_car_origin = nested_car

hand_brake = False
running = True
steering = 0
throttle = 0
brake = 0
print("theta: ")
print(theta)


start_time = time.time()
nested_car_origin.control(steering, throttle)
done = False
target_speed = 16.67  # m/s
for i in range(number_of_iterations):
    times = []
    x_positions_human_car = []
    y_positions_human_car = []
    x_positions_nested_car = []
    y_positions_nested_car = []
    x_positions_saddigh_car = []
    y_positions_saddigh_car = []
    steering_input = []
    input("Press Enter to start the experiment:")
    if done:
        nested_car_carla, human_car_carla = gt_vhcl(carla_world, carla_map)
    running = True
    first = True
    start = False
    human_cruise_flag = True
    # cntdwn(carla_world, human_car_carla)
    # time.sleep(5)
    while running:
        if nested_car_carla.get_location().x == 0:
            print("Cars were destroyed, waiting for new condition...")
            done = True
            running = False
        else:
            if nested_car_carla.get_location().x <= 367 and human_car_carla.get_location().x <= 367:
                if start is False:
                    start_time = time.time()
                    loop_time = start_time
                    start = True
                current_speed = nested_car_carla.get_velocities().x
                # Calculate the error (difference between current speed and target speed)
                error = target_speed - current_speed
                # Adjust throttle and brake based on the error
                if error > 0:
                    # Error is positive, accelerate
                    throttle = min(1.0, error / 5.0)  # Adjust the division factor for smoother acceleration
                elif error < 0:
                    # Error is negative, brake
                    brake = min(1.0, -error / 5.0)  # Adjust the division factor for smoother braking
                # Apply control inputs
                nested_car_carla.vehicle.apply_control(carla.VehicleControl(throttle=throttle, brake=brake, steer=0.0))
                human_car_carla.vehicle.apply_control(carla.VehicleControl(throttle=throttle, brake=brake, steer=0.0))
                carla_world.tick()
            elif nested_car_carla.get_location().x >= 367:
                loop_time = time.time()
                # nested_car.control(steering, throttle)
                print("------------------------------")
                if first:
                    first = False
                    nested_car.optimizer = nested_car_origin.optimizer
                    nested_car.control(steering, throttle)
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
                if human_cruise_flag is True:
                    human_cruise_flag = False
                    control = carla.VehicleControl(throttle=0, steer=0, brake=0,
                                                   hand_brake=0)
                    human_car_carla.vehicle.apply_control(control)
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
            x_positions_human_car.append(human_car_carla.get_location().x)
            y_positions_human_car.append(human_car_carla.get_location().y)
            x_positions_nested_car.append(nested_car_carla.get_location().x)
            y_positions_nested_car.append(nested_car_carla.get_location().y)
            x_positions_saddigh_car.append(nested_car.x[0])
            y_positions_saddigh_car.append(nested_car.x[1])
            steering_input.append(steering)
            times.append(carla_world.get_snapshot().timestamp.elapsed_seconds)
    end_time = time.time()
    title = title + str(i)
    pltng_scene(x_positions_human_car, y_positions_human_car, x_positions_nested_car, y_positions_nested_car, theta,
                start_time, end_time, title, times)
    title = title[:-1]
print("Experiment finished!")
