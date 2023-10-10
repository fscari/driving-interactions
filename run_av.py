import carla
import pandas as pd
import class_av
from plotting_scene import pltng_scene
from get_vehicles import gt_vhcl

def run_av(carla_world, human_car_carla, nested_car_carla, controller, vehicles_data):
    running = True
    steering = 0
    throttle = 0
    while running:
        if nested_car_carla.get_location().x == 0 or run is False:
            print("Cars were destroyed, waiting for new condition...")
            running = False
        if nested_car_carla.get_location().x >= 363:
            print("AV should take control...")
            print("x position: ", nested_car_carla.get_location().x)
            print("y position: ", nested_car_carla.get_location().y)
            print("steering: ", steering)
            print("throttle: ", throttle)
            print("y velocity: ", nested_car_carla.get_velocities().y)
            print("yaw: ", nested_car_carla.get_rotation().yaw)
            print("Yaw rate: ", nested_car_carla.get_angular_velocities().z)
            running = False

        throttle, steering = controller.control(nested_car_carla)
        # Apply control to the vehicle
        control = carla.VehicleControl(throttle=throttle, steer=steering)
        nested_car_carla.vehicle.apply_control(control)

        # Save position data
        new_data = {
            'times': carla_world.get_snapshot().timestamp.elapsed_seconds,
            'x_positions_human_car': human_car_carla.get_location().x,
            'y_positions_human_car': human_car_carla.get_location().y,
            'x_positions_nested_car': nested_car_carla.get_location().x,
            'y_positions_nested_car': nested_car_carla.get_location().y,
            'steering_input': steering
        }
        vehicles_data = pd.concat([vehicles_data, pd.DataFrame([new_data])], ignore_index=True)
    return  vehicles_data,  running


if __name__ == '__main__':
    client = carla.Client('131.180.28.182', 2000)  # Use the correct IP address and port
    client.set_timeout(10.0)  # Set a timeout value
    # Retrieve the list of vehicles
    carla_world = client.get_world()
    carla_map = carla_world.get_map()
    nested_car_carla, human_car_carla = gt_vhcl(carla_world, carla_map)
    target_speed_forCC = 16.67  # m/s
    look_ahead_dist_forCC = 15
    controller = class_av.AV(carla_map, nested_car_carla, target_speed_forCC, look_ahead_dist_forCC)
    theta = [controller.KPS, controller.KIS, controller.KDS, look_ahead_dist_forCC]
    vehicles_data = pd.DataFrame(columns=['times', 'x_positions_human_car', 'y_positions_human_car', 'x_positions_nested_car',
                               'y_positions_nested_car', 'steering_input'])
    running = True
    run = True
    while running:
        if nested_car_carla.get_location().x == 0 or run is False:
            print("Cars were destroyed, waiting for new condition...")
            running = False
        if human_car_carla.get_velocities().x > 0 and human_car_carla.get_location().x >= 50:
            vehicles_data, run = run_av(carla_world, human_car_carla, nested_car_carla, controller, vehicles_data)
    pltng_scene(vehicles_data, theta, 'testing CC', 1, 1)