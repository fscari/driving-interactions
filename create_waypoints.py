import carla


def create_waypoints(carla_map, nested_car_carla):
    # Define waypoints for the path
    start_end = [carla.Location(50, 17.34), carla.Location(300, 3.5), carla.Location(350, 2.5),
                 carla.Location(550, 1.75)]
    waypoint = carla_map.get_waypoint(nested_car_carla.get_location())
    distance_between_waypoints = 2
    waypoints = []
    while waypoint.transform.location.distance(start_end[1]) > distance_between_waypoints:
        # Move to the next waypoint
        waypoint = carla_map.get_waypoint(waypoint.next(distance_between_waypoints)[0].transform.location)
        waypoints.append(waypoint)
    while waypoint.transform.location.distance(start_end[2]) > distance_between_waypoints:
        # Move to the next waypoint
        waypoint = carla_map.get_waypoint(waypoint.next(distance_between_waypoints)[0].transform.location)
        waypoints.append(waypoint)
    while waypoint.transform.location.distance(start_end[3]) > distance_between_waypoints:
        # Move to the next waypoint
        waypoint = carla_map.get_waypoint(waypoint.next(distance_between_waypoints)[0].transform.location)
        waypoints.append(waypoint)

    return waypoints
