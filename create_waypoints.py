import carla


def create_waypoints(carla_map, nested_car_carla):
    # Define waypoints for the path
    start_end = [carla.Location(nested_car_carla.get_location().x, nested_car_carla.get_location().y),
                 carla.Location(265, 5.75), carla.Location(275, 4.9),
                 carla.Location(285, 4.4), carla.Location(295, 3.9),
                 carla.Location(305, 3.4), carla.Location(315, 3),
                 carla.Location(325, 2.8), carla.Location(335, 2.5),
                 carla.Location(345, 2.3), carla.Location(355, 2),
                 carla.Location(365, 1.8), carla.Location(366, 1.75),
                 carla.Location(550, 1.75)]
    waypoint = carla_map.get_waypoint(nested_car_carla.get_location())
    distance_between_waypoints = 0.5
    waypoints = []
    for i in range(1, len(start_end)):
        while waypoint.transform.location.distance(start_end[i]) > distance_between_waypoints:
            # Move to the next waypoint
            waypoint = carla_map.get_waypoint(waypoint.next(distance_between_waypoints)[0].transform.location)
            waypoints.append(waypoint)
    return waypoints
