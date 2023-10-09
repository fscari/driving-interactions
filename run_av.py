import carla
import pandas as pd


def run_av(carla_world, human_car_carla, nested_car_carla, controller, vehicles_data):
    running = True
    steering = 0
    throttle = 0
    while running:
        if nested_car_carla.get_location().x >= 365:
            print("AV should take control...")
            print("x position: ", nested_car_carla.get_location().x)
            print("y position: ", nested_car_carla.get_location().y)
            print("steering: ", steering)
            print("throttle: ", throttle)
            print("y velocity: ", nested_car_carla.get_velocities().y)
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
    return  vehicles_data