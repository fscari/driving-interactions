import carla
import time


def cntdwn(world, human_car_carla):
# Set the countdown duration and start the countdown
    initial_y = -2
    duration =5.0
    initial_x = human_car_carla.location.x + 10
    for i in range(5):
        current_y = initial_y + i
        current_time = duration - i
        point_location = carla.Location(x=initial_x, y=current_y, z=1.3)
        world.debug.draw_point(point_location, size=0.5, life_time=current_time)
