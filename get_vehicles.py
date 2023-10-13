import sys
import joaninit


def gt_vhcl(carla_world, carla_map):
    vehicle_list = carla_world.get_actors().filter('vehicle.*')
    if vehicle_list:
        for vehicle in vehicle_list:
            if vehicle.type_id == 'vehicle.hapticslab.nissannpc':
                nested_car_carla = joaninit.Vehicle(vehicle, carla_map)
            elif vehicle.type_id == 'vehicle.hapticslab.nissanego':
                human_car_carla = joaninit.Vehicle(vehicle, carla_map)
            else:
                fede_car_carla = joaninit.Vehicle(vehicle, carla_map)
    else:
        print("No vehicles found")
        sys.exit()

    return nested_car_carla, human_car_carla, fede_car_carla
