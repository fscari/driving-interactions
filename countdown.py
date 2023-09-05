import carla
import time

def display_countdown(countdown_value, client):
    world = client.get_world()
    display_locations = [
        carla.Location(x=52, y=1.3, z=1.3),
        carla.Location(x=52, y=-1.3, z=1.3),
        carla.Location(x=-52, y=1.3, z=1.3),
        carla.Location(x=-52, y=-1.3, z=1.3),
    ]

    for location in display_locations:
        world.debug.draw_string(
            location,
            f"Countdown: {countdown_value}",
            color=carla.Color(r=255, g=0, b=0),
            life_time=0.1  # Display time for the text
        )

# Connect to the Carla server
client = carla.Client('131.180.29.49', 2000)  # Use the correct IP address and port
client.set_timeout(10.0)
# def cntdwn(client):
# Set the countdown duration and start the countdown
countdown_duration = 10  # in seconds
current_countdown = countdown_duration

while current_countdown >= 0:
    display_countdown(current_countdown, client)
    current_countdown -= 1
    time.sleep(1)  # Wait for 1 second before updating the countdown

##
# actor_id = 12  # Example actor ID
# actor = world.get_actor(actor_id)
#
# if actor is not None:
#     print("Actor ID:", actor.id)
#     print("Actor Type:", actor.type_id)
#
#     # Print all attributes and their values
#     for key, value in actor.attributes.items():
#         print(f"{key}: {value}")
# else:
#     print("Actor not found.")