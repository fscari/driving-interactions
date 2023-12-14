import sys
import joaninit


def gt_vhcl(carla_world, carla_map, vehicle_nr):
    vehicle_list = carla_world.get_actors().filter('vehicle.*')
    if vehicle_list:
        if vehicle_nr == 1:
            for vehicle in vehicle_list:
                if vehicle.type_id == 'vehicle.hapticslab.nissanlinux1':
                    nested_car_carla = joaninit.Vehicle(vehicle, carla_map)
                    break
            for vehicle in vehicle_list:
                if vehicle.type_id != 'vehicle.hapticslab.nissanlinux1' and vehicle.type_id != 'vehicle.hapticslab.nissanlinux2' and vehicle.type_id != 'vehicle.hapticslab.audi':
                    current_vehicle = joaninit.Vehicle(vehicle, carla_map)
                    if nested_car_carla.get_location().x > 0 and current_vehicle.get_location().x > 0:
                        if vehicle.type_id == 'vehicle.hapticslab.nissanego_fede':
                            human_car_carla = joaninit.Vehicle(vehicle, carla_map)
                            condition_M = 0
                            break
                        elif vehicle.type_id == 'vehicle.hapticslab.nissannpc_fede':
                            human_car_carla = joaninit.Vehicle(vehicle, carla_map)
                            condition_M = 1
                            break
            if nested_car_carla.get_location().y < 0:
                condition_id = 0
            elif nested_car_carla.get_location().y > 0:
                condition_id = 1
        elif vehicle_nr == 2:
            for vehicle in vehicle_list:
                if vehicle.type_id == 'vehicle.hapticslab.nissanlinux2':
                    nested_car_carla = joaninit.Vehicle(vehicle, carla_map)
                    break
            for vehicle in vehicle_list:
                if vehicle.type_id != 'vehicle.hapticslab.nissanlinux1' and vehicle.type_id != 'vehicle.hapticslab.nissanlinux2' and vehicle.type_id != 'vehicle.hapticslab.audi':
                    current_vehicle = joaninit.Vehicle(vehicle, carla_map)
                    if nested_car_carla.get_location().x < 0 and current_vehicle.get_location().x < 0:
                        if vehicle.type_id == 'vehicle.hapticslab.nissanego_fede':
                            human_car_carla = joaninit.Vehicle(vehicle, carla_map)
                            condition_M = 1
                            break
                        elif vehicle.type_id == 'vehicle.hapticslab.nissannpc_fede':
                            human_car_carla = joaninit.Vehicle(vehicle, carla_map)
                            condition_M = 0
                            break
            if nested_car_carla.get_location().y > 0:
                condition_id = 0
            elif nested_car_carla.get_location().y < 0:
                condition_id = 1
        for vehicle in vehicle_list:
            if vehicle.type_id == 'vehicle.hapticslab.audi':
                fede_car_carla = joaninit.Vehicle(vehicle, carla_map)
                break
    else:
        print("No vehicles found")
        sys.exit()

    return nested_car_carla, human_car_carla, fede_car_carla, condition_id, condition_M
