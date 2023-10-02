import carla
import time

# # Connect to the Carla server
# client = carla.Client('131.180.29.49', 2000)  # Use the correct IP address and port
# client.set_timeout(10.0)
def cntdwn(client, world):
# Set the countdown duration and start the countdown
    initial_y = -2
    duration =5.0

    for i in range(5):
        current_y = initial_y + i
        current_time = duration - i
        point_location = carla.Location(x=60, y=current_y, z=1.3)
        world.debug.draw_point(point_location, size=0.5, life_time=current_time)
