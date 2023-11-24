import sys
import joaninit


def gt_vhcl_M(carla_world, carla_map, vehicle_nr):
    vehicle_list = carla_world.get_actors().filter('vehicle.*')
    if vehicle_list:
        if vehicle_nr == 1:
            for vehicle in vehicle_list:
                if vehicle.type_id == 'vehicle.hapticslab.nissanlinux1':
                    nested_car_carla = joaninit.Vehicle(vehicle, carla_map)
                elif vehicle.type_id == 'vehicle.hapticslab.nissannpc_fede':
                    human_car_carla = joaninit.Vehicle(vehicle, carla_map)
                elif vehicle.type_id == 'vehicle.hapticslab.audi':
                    fede_car_carla = joaninit.Vehicle(vehicle, carla_map)
        elif vehicle_nr == 2:
            for vehicle in vehicle_list:
                if vehicle.type_id == 'vehicle.hapticslab.nissanlinux2':
                    nested_car_carla = joaninit.Vehicle(vehicle, carla_map)
                elif vehicle.type_id == 'vehicle.hapticslab.nissanego_fede':
                    human_car_carla = joaninit.Vehicle(vehicle, carla_map)
                elif vehicle.type_id == 'vehicle.hapticslab.audi':
                    fede_car_carla = joaninit.Vehicle(vehicle, carla_map)
    else:
        print("No vehicles found")
        sys.exit()

    return nested_car_carla, human_car_carla, fede_car_carla
