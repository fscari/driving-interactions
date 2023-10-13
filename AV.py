import  carla
import math
import joaninit
import pickle
import os


# Constants
TARGET_SPEED = 16.67  # m/s
LOOKAHEAD_DISTANCE = 15  # meters

# PID constants for longitudinal control
KPT= 0.1
KIT = 0.1
KDT = 1
# PID constants for lateral control
KPS = 0.001
KIS = 0.001 #0.001
KDS = 0.01  #0.01

class AV:
    def __init__(self):
        self.integral_error = 0
        self.prev_error = 0
        self.prev_yaw_error = 0
        self.integral_yaw_error = 0

    def longitudinal_control(self, current_speed):
        error = TARGET_SPEED - current_speed
        self.integral_error += error
        if abs(self.integral_error) > TARGET_SPEED * 10:
            self.integral_error = 0

        derivative_error = error - self.prev_error
        throttle = KPT * error + KIT * self.integral_error + KDT * derivative_error
        self.prev_error = error
        return min(max(throttle, 0), 0.5)  # Clamp between 0 and 1

    def lateral_control(self, vehicle_location, vehicle_rotation, next_waypoint):
        # Calculate the desired heading using the lookahead point
        dx = next_waypoint.transform.location.x - vehicle_location.x
        dy = next_waypoint.transform.location.y - vehicle_location.y
        desired_yaw = math.atan2(dy, dx)
        # Calculate the current vehicle heading
        vehicle_yaw = math.radians(vehicle_rotation.yaw)

        # Calculate the error between desired and current heading
        yaw_error = desired_yaw - vehicle_yaw
        yaw_error_rate = yaw_error - self.prev_yaw_error
        self.prev_yaw_error = yaw_error
        MAX_INTEGRAL_YAW_ERROR = 4.0
        self.integral_yaw_error = min(max(self.integral_yaw_error, -MAX_INTEGRAL_YAW_ERROR), MAX_INTEGRAL_YAW_ERROR)
        # steering = 0.0005 * yaw_error  # The factor 2.0 can be tuned
        steering = KPS * yaw_error + KIS * self.integral_yaw_error - KDS * yaw_error_rate # The 0.5 value can be tuned
        return min(max(steering, -1), 1)  # Clamp between -1 and 1

    def get_next_waypoint(self, vehicle_location, waypoints):
        # Find the waypoint that's closest to the lookahead distance
        min_distance = float('inf')
        next_waypoint = None
        for waypoint in waypoints:
            distance = math.sqrt((waypoint.transform.location.x - vehicle_location.x)**2 + (waypoint.transform.location.y - vehicle_location.y)**2)
            if distance < LOOKAHEAD_DISTANCE and distance < min_distance:
                min_distance = distance
                next_waypoint = waypoint
        return next_waypoint

    def control(self, vehicle_location, vehicle_rotation,  current_speed, waypoints):
        next_waypoint = self.get_next_waypoint(vehicle_location, waypoints)
        print(next_waypoint)
        if not next_waypoint:
            return 0, 0  # No control if no waypoints are found

        throttle = self.longitudinal_control(current_speed)
        steering = self.lateral_control(vehicle_location, vehicle_rotation, next_waypoint)

        return throttle, steering

# # Assuming you have a Carla client and vehicle actor
# client = carla.Client('131.180.28.182', 2000)  # Use the correct IP address and port
# client.set_timeout(10.0)  # Set a timeout value
# # Retrieve the list of vehicles
# carla_world = client.get_world()
# carla_map = carla_world.get_map()
# vehicle = carla_world.get_actors().filter('vehicle.*')
# nested_car_carla = joaninit.Vehicle(vehicle[0], carla_map)
#
# controller = AV()
#
# # Define waypoints for the path
# start_end = [carla.Location(50, 17.34), carla.Location(300, 3.5), carla.Location(350,1.75), carla.Location(550,1.75)]
# waypoint = carla_map.get_waypoint(nested_car_carla.get_location())
# distance_between_waypoints = 2
# waypoints = []
# filename = "waypoints.pkl"
# if os.path.exists(filename):
#     # Load the list back from the file
#     with open(filename, 'rb') as file:
#         waypoints = pickle.load(file)
# else:
#     while waypoint.transform.location.distance(start_end[1]) > distance_between_waypoints:
#         # Move to the next waypoint
#         waypoint = carla_map.get_waypoint(waypoint.next(distance_between_waypoints)[0].transform.location)
#         waypoints.append(waypoint)
#     while waypoint.transform.location.distance(start_end[2]) > distance_between_waypoints:
#         # Move to the next waypoint
#         waypoint = carla_map.get_waypoint(waypoint.next(distance_between_waypoints)[0].transform.location)
#         waypoints.append(waypoint)
#     while waypoint.transform.location.distance(start_end[3]) > distance_between_waypoints:
#     # Move to the next waypoint
#         waypoint = carla_map.get_waypoint(waypoint.next(distance_between_waypoints)[0].transform.location)
#         waypoints.append(waypoint)
#     with open(filename, 'wb') as file:
#         pickle.dump(waypoints, file)



running = True
while running:
    if nested_car_carla.get_location().x == 0:
        print("Cars were destroyed, waiting for new condition...")
        done = True
        running = False
    elif nested_car_carla.get_location().x >= 365:
        print("AV should take control...")
        print("x position: ", nested_car_carla.get_location().x)
        print("y position: ", nested_car_carla.get_location().y)
        print("steering: ", steering)
        print("throttle: ", throttle)
        running = False
    vehicle_location = nested_car_carla.get_location()
    vehicle_rotation = nested_car_carla.get_rotation()
    current_speed = nested_car_carla.get_velocity()
    # current_speed = math.sqrt(vehicle_speed.x**2 + vehicle_speed.y**2 + vehicle_speed.z**2)

    throttle, steering = controller.control(vehicle_location, vehicle_rotation, current_speed, waypoints)

    # Apply control to the vehicle
    control = carla.VehicleControl(throttle=throttle, steer=steering)
    nested_car_carla.vehicle.apply_control(control)
