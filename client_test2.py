import sys
import carla
import time
from countdown import cntdwn
import joaninit
from controller_init import cntrlr_init
from plotting_scene import pltng_scene

# Connect to the CARLA server
client = carla.Client('131.180.28.182', 2000)  # Use the correct IP address and port
client.set_timeout(10.0)  # Set a timeout value
# Retrieve the list of vehicles
carla_world = client.get_world()
carla_map = carla_world.get_map()
vehicle_list = carla_world.get_actors().filter('vehicle.*')
if vehicle_list:
    for vehicle in vehicle_list:
        if vehicle.type_id == 'vehicle.hapticslab.nissannpc':
            nested_car_carla = joaninit.Vehicle(vehicle, carla_map)
        else:
            human_car_carla = joaninit.Vehicle(vehicle, carla_map)
else:
    print("No vehicles found")
    sys.exit()
# Saddigh
dt = 0.01
theta = [5, -46.271, 90.15, 8.531, -100.604]
T = 10
human_car, nested_car = cntrlr_init(dt, human_car_carla, nested_car_carla, theta)
nested_car_origin = nested_car

hand_brake = False
running = True
steering = 0
throttle = 0
print("theta: ")
print(theta)

times = []
x_positions_human_car = []
y_positions_human_car = []
x_positions_nested_car = []
y_positions_nested_car = []
x_positions_saddigh_car = []
y_positions_saddigh_car = []
steering_input = []
number_of_experiments = 10
start_time = time.time()
nested_car_origin.control(steering, throttle)
done = False
for i in range(number_of_experiments):
    input("Press Enter to start:")
    if done:
        vehicle_list = carla_world.get_actors().filter('vehicle.*')
        if vehicle_list:
            for vehicle in vehicle_list:
                if vehicle.type_id == 'vehicle.hapticslab.nissannpc':
                    nested_car_carla = joaninit.Vehicle(vehicle, carla_map)
                else:
                    human_car_carla = joaninit.Vehicle(vehicle, carla_map)
        else:
            print("No vehicles found")
            sys.exit()
    running = True
    cntdwn(client, carla_world)
    time.sleep(5)
    first = True
    # u = nested_car.traj.u[0].get_value()
    while running:
        if nested_car_carla.get_location().x == 0:
            print(nested_car_carla.get_location().x)
            done = True
            running = False
        elif vehicle_list:
            start_time = time.time()
            nested_car.control(steering, throttle)
            print("------------------------------")
            if first:
                first = False
                nested_car.optimizer = nested_car_origin.optimizer
            else:
                nested_car.control(steering, throttle)
            u = nested_car.traj.u[0].get_value()
            steering = u[0]
            throttle = u[1]
            if throttle < 0:
                brake = abs(throttle)
                throttle = 0
            else:
                brake = 0
            # Set control commands
            throttle = throttle  # Set the desired throttle value (0 to 1)
            steer = steering  # Set the desired steering value (-1 to 1)
            brake = brake  # Set the desired brake value (0 to 1)
            control = carla.VehicleControl(throttle=throttle, steer=steer, brake=brake, hand_brake=hand_brake)
            # Apply the control commands to the vehicle
            nested_car_carla.vehicle.apply_control(control)

            carla_world.tick()
            nested_car.move(nested_car_carla, human_car_carla)
            human_car.move(human_car_carla)
            print("input: ")
            print(steering, throttle)
            print("state: ")
            print(nested_car_carla.get_location().x, nested_car_carla.get_location().y, nested_car_carla.get_rotation().yaw, nested_car_carla.get_velocity())

            sleep_time = dt - (time.time() - start_time)
            if sleep_time > 0:
                time.sleep(sleep_time)

            x_positions_human_car.append(human_car_carla.get_location().x)
            y_positions_human_car.append(human_car_carla.get_location().y)
            x_positions_nested_car.append(nested_car_carla.get_location().x)
            y_positions_nested_car.append(nested_car_carla.get_location().y)
            x_positions_saddigh_car.append(nested_car.x[0])
            y_positions_saddigh_car.append(nested_car.x[1])
            steering_input.append(steer)
            times.append(time.time() - start_time)


end_time = time.time()
theta.append(T)
pltng_scene(x_positions_human_car, y_positions_human_car, x_positions_nested_car, y_positions_nested_car, theta, start_time, end_time)