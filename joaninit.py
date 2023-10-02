import carla
import math


class Vehicle:
    def __init__(self, car, carla_map):
        self.vehicle = car
        self.map = carla_map
        self.transform = car.get_transform()
        self.location = self.transform.location
        self.rotation = self.transform.rotation
        self.velocities = car.get_velocity()
        self.velocity = math.sqrt(car.get_velocity().x**2 + car.get_velocity().y**2)
        self.angular_velocities = car.get_angular_velocity()
        self.waypoint = carla_map.get_waypoint(self.location, project_to_road=True, lane_type=carla.LaneType.Driving)
    def get_transform(self):
        return self.vehicle.get_transform()
    def get_velocities(self):
        return self.vehicle.get_velocity()
    def get_velocity(self):
        self.velocity = math.sqrt(self.vehicle.get_velocity().x**2 + self.vehicle.get_velocity().y**2)
        return self.velocity
    def get_location(self):
        return self.vehicle.get_transform().location
    def get_rotation(self):
        return self.vehicle.get_transform().rotation
    def get_angular_velocities(self):
        return self.vehicle.get_angular_velocity()
    def get_waypoint(self):
        return self.map.get_waypoint(self.location, project_to_road=True, lane_type=carla.LaneType.Driving)
