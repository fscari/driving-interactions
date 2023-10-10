import math
import carla
import pickle
import os
from create_waypoints import create_waypoints

class AV:
    def __init__(self, carla_map, nested_car_carla, target_speed, look_ahead_dist):
        # Create waypoints
        self.waypoints = create_waypoints(carla_map, nested_car_carla)
        self.target_speed = target_speed
        self.look_ahead_dist = look_ahead_dist
        # errors
        self.integral_error = 0
        self.prev_error = 0
        self.prev_yaw_error = 0
        self.integral_yaw_error = 0
        self.max_integral_yaw_error = 4
        # PID constants for longitudinal control
        self.KPT = 10
        self.KIT = 0.1
        self.KDT = 1
        # PID constants for lateral control
        self.KPS = 0.001
        self.KIS = 0.0001 #0.01
        self.KDS = 1000
        # self.KPS = 0.001
        # self.KIS = 0.01
        # self.KDS = 0.1

    def longitudinal_control(self, current_speed):
        error = self.target_speed - current_speed
        self.integral_error += error
        if abs(self.integral_error) > self.target_speed * 10:
            self.integral_error = 0

        derivative_error = error - self.prev_error
        throttle = self.KPT * error + self.KIT * self.integral_error + self.KDT * derivative_error
        self.prev_error = error
        return min(max(throttle, 0), 0.9)  # Clamp between 0 and 1

    def lateral_control(self, vehicle_location, vehicle_rotation, next_waypoint):
        # Calculate the desired heading using the lookahead point
        dx = next_waypoint.transform.location.x - vehicle_location.x
        dy = next_waypoint.transform.location.y - vehicle_location.y
        # print("lateral offset: ", dy)
        desired_yaw = math.atan2(dy, dx)
        # Calculate the current vehicle heading
        vehicle_yaw = math.radians(vehicle_rotation.yaw)
        # Calculate the error between desired and current heading
        yaw_error = desired_yaw - vehicle_yaw
        yaw_error_rate = yaw_error - self.prev_yaw_error
        self.prev_yaw_error = yaw_error
        self.integral_yaw_error = min(max(self.integral_yaw_error, -self.max_integral_yaw_error), self.max_integral_yaw_error)
        steering = self.KPS * yaw_error + self.KIS * self.integral_yaw_error - self.KDS * yaw_error_rate
        return min(max(steering, -1), 1)  # Clamp between -1 and 1

    def get_next_waypoint(self, vehicle_location):
        # Find the waypoint that's closest to the lookahead distance
        min_distance = float('inf')
        next_waypoint = None
        for waypoint in self.waypoints:
            distance = math.sqrt((waypoint.transform.location.x - vehicle_location.x)**2 + (waypoint.transform.location.y - vehicle_location.y)**2)
            if distance < self.look_ahead_dist and distance < min_distance:
                min_distance = distance
                next_waypoint = waypoint
        return next_waypoint

    def control(self, nested_car_carla):
        next_waypoint = self.get_next_waypoint(nested_car_carla.get_location())
        if not next_waypoint:
            return 0, 0  # No control if no waypoints are found
        throttle = self.longitudinal_control(nested_car_carla.get_velocity())
        steering = self.lateral_control(nested_car_carla.get_location(), nested_car_carla.get_rotation(), next_waypoint)

        return throttle, steering
