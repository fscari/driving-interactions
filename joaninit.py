import carla
import math


class Vehicle:
    def __init__(self, car, carla_map):
        self.transform = car.get_transform()
        self.location = self.transform.location
        self.rotation = self.transform.rotation
        self.velocities = car.get_velocity()
        self.velocity = math.sqrt(self.velocities.x**2 + self.velocities.y**2)
        self.angular_velocities = car.get_angular_velocity()
        self.waypoint = carla_map.get_waypoint(self.location,project_to_road=True, lane_type=(carla.LaneType.Driving))
    def get_transform(self):
        return self.transform
    def get_velocities(self):
        return self.velocities

    def get_velocity(self):
        return self.velocity

    def get_location(self):
        return self.location

    def get_rotation(self):
        return self.rotation

    def get_angular_velocities(self):
        return self.waypoiangular_velocitiesnt

    def get_waypoint(self):
        return self.waypoint