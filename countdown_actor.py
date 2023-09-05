import carla
from carla import TrafficLightState, TrafficLight

def create_countdown_traffic_light(world, location):
    blueprint_library = world.get_blueprint_library()
    traffic_light_bp = blueprint_library.find('traffic.traffic_light')
    traffic_light_bp.set_attribute('State', 'Red')  # Start with red light
    traffic_light = world.spawn_actor(traffic_light_bp, location)
    return traffic_light

def update_traffic_light_state(traffic_light, timer_value):
    if timer_value > 20:  # Example threshold for green state
        traffic_light.set_state(TrafficLightState.Green)
    elif timer_value > 10:  # Example threshold for yellow state
        traffic_light.set_state(TrafficLightState.Yellow)
    else:
        traffic_light.set_state(TrafficLightState.Red)


client = carla.Client('131.180.29.49', 2000)  # Use the correct IP address and port
client.set_timeout(10.0)
timer_value = get_current_timer_value()  # Replace with your timer logic
update_traffic_light_state(traffic_light, timer_value)

traffic_light.destroy()
