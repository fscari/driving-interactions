import carla
import dynamics, lane, car, feature, world
import math
import numpy as np
import matplotlib.pyplot as plt
import time
from countdown import cntdwn
import joaninit

# Connect to the CARLA server
client = carla.Client('131.180.28.182', 2000)  # Use the correct IP address and port
client.set_timeout(10.0)  # Set a timeout value
# Retrieve the list of vehicles
carla_world = client.get_world()
carla_map = carla_world.get_map()
vehicle_list = carla_world.get_actors().filter('vehicle.*')
for vehicle in vehicle_list:
    if vehicle.type_id == 'vehicle.hapticslab.nissannpc':
        nestedcar_carla = vehicle
        # nestedcar_carla = joaninit.Vehicle(vehicle)
    else:
        humancar_carla = vehicle
humancar_transform = humancar_carla.get_transform()
humancar_velocities = humancar_carla.get_velocity()
humancar_velocity = math.sqrt(humancar_velocities.x**2 + humancar_velocities.y**2)
nestedcar_transform = nestedcar_carla.get_transform()
nested_velocities = nestedcar_carla.get_velocity()
nested_velocity = math.sqrt(nested_velocities.x**2 + nested_velocities.y**2)
humancar_carla_waypoint = carla_world.get_map().get_waypoint(humancar_carla.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving))
nestedcar_carla_waypoint = carla_world.get_map().get_waypoint(nestedcar_carla.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving))

# Saddigh
dt = 0.01
dyn = dynamics.CarDynamics(dt)
humancar = car.UserControlledCar(dyn, [humancar_transform.location.x, humancar_transform.location.y, math.radians(humancar_transform.rotation.yaw), humancar_velocities.x, humancar_velocities.y, humancar_carla.get_angular_velocity().z], color='yellow')
nestedcar = car.NestedOptimizerCar(dyn, [nestedcar_transform.location.x, nestedcar_transform.location.y, math.radians(nestedcar_transform.rotation.yaw), nested_velocities.x, nested_velocities.y, nestedcar_carla.get_angular_velocity().z], color='yellow')
# humancar = car.UserControlledCar(dyn, [humancar_transform.location.x, humancar_transform.location.y, math.radians(humancar_transform.rotation.yaw), humancar_velocity, ], color='yellow')
# nestedcar = car.NestedOptimizerCar(dyn, [nestedcar_transform.location.x, nestedcar_transform.location.y, math.radians(nestedcar_transform.rotation.yaw), nested_velocity], color='yellow')
experiment_env = world.World()
experiment_env.cars.append(humancar)
experiment_env.cars.append(nestedcar)
experiment_env.cars[1].human = experiment_env.cars[0]
# nestedcar.human = humancar
clane1 = lane.StraightLane([50, -1.75], [460, -1.75], 3.5)
clane2 = lane.StraightLane([50, 1.75], [330, 1.75], 3.5)
# fakelane = lane.StraightLane([330.5, 3.5], [350, 0.1], 1)
fakelane = lane.StraightLane([400, 1.75], [460, 1.75], 3.5)#381
experiment_env.lanes += [clane1, clane2]
experiment_env.roads += [clane1] #, clane2] #[clane1.shifted(0.5), clane1.shifted(-1.5)]
# experiment_env.fences += [clane1.shifted(-1), clane2.shifted(1)]
experiment_env.fences += [clane1.shifted(-1), clane2.shifted(1), fakelane.shifted(0.5)] #, clane1.shifted()]

# nestedcar.human = humancar

flag = True
# humancar.reward = experiment_env.simple_reward(humancar, speed=humancar_velocity)
        # Choose the first vehicle
        #theta = [lanes, fance, road, speed, trajectoy.h]
# theta = [100, -10, 100, 1, -100] # works fine
# theta = [0.9, -46.271, 9.015, 8.531, -57.604] #sadighs
theta = [5, -46.271, 90.15, 8.531, -100.604]
T = 10
# r_h = experiment_env.simple_reward([nestedcar.traj], theta, speed_import=22 if flag else 1.,
#                                            speed=22 if flag else 1.) + 100. * feature.bounded_control(humancar.bounds)
r_h = experiment_env.simple_reward([nestedcar.traj], theta)+100.*feature.bounded_control(humancar.bounds)
# h_speed = humancar_velocity
r_r = experiment_env.simple_reward(nestedcar, theta, speed=22) # 300. * h_speed +
experiment_env.cars[1].rewards = (r_h, r_r)

hand_brake = False
running = True
count = 0
steering = 0
throttle = 0
print("theta: ")
print(theta)

times = []
y_positions_nestedcar = []
y_positions_humancar = []
x_positions_nestedcar = []
x_positions_humancar = []
steering_input = []

y_positions_sadcar = []
x_positions_sadcar = []
start_time = time.time()



first = True
while running:
    if nestedcar_transform.location.x == 0:
        running = False
    elif vehicle_list:
        start_time = time.time()
        nestedcar.control(steering, throttle)
        print("------------------------------")
        if first:
            cntdwn(client, carla_world)
            time.sleep(5)
            first = False
        u = nestedcar.traj.u[0].get_value()
        steering = u[0]
        throttle = u[1]
        if throttle < 0:
            brake = abs(throttle)
            throttle = 0
        else:
            brake = 0
        # Set control commands
        throttle = throttle  # Set the desired throttle value (0 to 1)
        steer = steering  # Set the desired steering value (-1 to 1)
        brake = brake  # Set the desired brake value (0 to 1)
        control = carla.VehicleControl(throttle=throttle, steer=steer, brake=brake, hand_brake=hand_brake)
        # Apply the control commands to the vehicle
        nestedcar_carla.apply_control(control)

        carla_world.tick()
        nestedcar.move(nestedcar_carla, humancar_carla)
        humancar.move(humancar_carla)
        nestedcar_transform = nestedcar_carla.get_transform()
        humancar_transform = humancar_carla.get_transform()
        print("input: ")
        print(steering, throttle)
        print("state: ")
        print(nestedcar_transform.location.x, nestedcar_transform.location.y, nestedcar_transform.rotation.yaw, nested_velocity)

        elapsed_time = time.time() - start_time  # get the current time value
        sleep_time = dt - elapsed_time
        if sleep_time > 0:
            time.sleep(sleep_time)
        current_y_position_nestedcar = nestedcar_transform.location.y  # assuming the second state value is the lateral position
        current_y_position_humancar = humancar_transform.location.y
        current_x_position_nestedcar = nestedcar_transform.location.x  # assuming the second state value is the lateral position
        current_x_position_humancar = humancar_transform.location.x

        current_y_position_sadcar = nestedcar.x[1]
        current_x_position_sadcar = nestedcar.x[0]

        current_steering_input = steer
        times.append(elapsed_time)
        y_positions_nestedcar.append(current_y_position_nestedcar)
        y_positions_humancar.append(current_y_position_humancar)
        x_positions_nestedcar.append(current_x_position_nestedcar)
        x_positions_humancar.append(current_x_position_humancar)
        steering_input.append(current_steering_input)

        y_positions_sadcar.append(current_y_position_sadcar)
        x_positions_sadcar.append(current_x_position_sadcar)
        # count +=1
end_time = time.time()
theta.append(T)
## plot
# Create a figure with two subplots
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))  # 1 row, 2 columns
# Left fence coordinates
left_fence_x = [51, 410]
left_fence_y = [-3.5, -3.5]
# Right fence coordinates
right_fence_x1 = [51, 330]
right_fence_y1 = [3.5, 3.5]
right_fence_x2 = [330, 350]
right_fence_y2 = [3.5, 0]
right_fence_x3 = [350, 410]
right_fence_y3 = [0, 0]
# Plotting
# ax1.figure(figsize=(10,6))
ax1.plot(left_fence_x, left_fence_y, color='black') #label='Left Fence')
ax1.plot(right_fence_x1, right_fence_y1, color='black') # label='Right Fence Segment 1'
ax1.plot(right_fence_x2, right_fence_y2, color='black') # label='Right Fence Segment 2'
ax1.plot(right_fence_x3, right_fence_y3, color='black') # label='Right Fence Segment 3'
# Plotting the middle dashed line
ax1.axhline(0, color='black', linestyle='--', xmin=41/410, xmax=310/410)  # the xmin and xmax values are normalized

filtered_x_positions_humancar = []
filtered_y_positions_humancar = []
for x_val, y_val in zip(x_positions_humancar, y_positions_humancar):
    if x_val != 0 or y_val != 0:
        filtered_x_positions_humancar.append(x_val)
        filtered_y_positions_humancar.append(y_val)

filtered_x_positions_nestedcar = []
filtered_y_positions_nestedcar = []
for x_val, y_val in zip(x_positions_nestedcar, y_positions_nestedcar):
    if x_val != 0 or y_val != 0:
        filtered_x_positions_nestedcar.append(x_val)
        filtered_y_positions_nestedcar.append(y_val)

ax1.plot(filtered_x_positions_nestedcar[:], filtered_y_positions_nestedcar[:], color='green', label='nestedcar', linewidth=2)
ax1.plot(filtered_x_positions_humancar[:], filtered_y_positions_humancar[:], color='red', label='humancar', linewidth=2)
# Additional plot settings
ax1.set_xlabel('X Position')
ax1.set_ylabel('Y Position')
ax1.set_title(theta)
ax1.grid(False)
ax1.invert_yaxis()
ax1.legend()


total_time = int((end_time-start_time)*100)
ratio = int(len(filtered_x_positions_nestedcar)/total_time)
filtered_x_positions_nestedcar_pertime = np.zeros(total_time)
filtered_y_positions_nestedcar_pertime = np.zeros(total_time)
filtered_x_positions_humancar_pertime = np.zeros(total_time)
filtered_y_positions_humancar_pertime = np.zeros(total_time)
for i in range(total_time):
    filtered_x_positions_nestedcar_pertime[i] = filtered_x_positions_nestedcar[i * ratio]
    filtered_y_positions_nestedcar_pertime[i] = filtered_y_positions_nestedcar[i * ratio]
    filtered_x_positions_humancar_pertime[i] = filtered_x_positions_humancar[i * ratio]
    filtered_y_positions_humancar_pertime[i] = filtered_y_positions_humancar[i * ratio]
# ax2.figure(figsize=(10,6))
ax2.plot(left_fence_x, left_fence_y, color='black') #label='Left Fence')
ax2.plot(right_fence_x1, right_fence_y1, color='black') # label='Right Fence Segment 1'
ax2.plot(right_fence_x2, right_fence_y2, color='black') # label='Right Fence Segment 2'
ax2.plot(right_fence_x3, right_fence_y3, color='black') # label='Right Fence Segment 3'
# Plotting the middle dashed line
ax2.axhline(0, color='black', linestyle='--', xmin=41/410, xmax=310/410)  # the xmin and xmax values are normalized
ax2.scatter(filtered_x_positions_nestedcar_pertime[:], filtered_y_positions_nestedcar_pertime[:], color='green', label='nestedcar', linewidth=2)
ax2.scatter(filtered_x_positions_humancar_pertime[:], filtered_y_positions_humancar_pertime[:], color='red', label='humancar', linewidth=2)
# Additional plot settings
ax2.set_xlabel('X Position')
ax2.set_ylabel('Y Position')
ax2.set_title(theta)
ax2.grid(False)
ax2.invert_yaxis()
ax2.legend()
plt.show()


# plt.plot(x_positions_sadcar, y_positions_sadcar, color='red', label='sadcar', linewidth=2)
# plt.plot(x_positions_nestedcar[:-1], y_positions_nestedcar[:-1], color='green', label='nestedcar', linewidth=2)
# plt.legend()
# plt.show()
#
# plt.figure(figsize=(10,6))
# plt.plot(left_fence_x, left_fence_y, color='black') #label='Left Fence')
# plt.plot(right_fence_x1, right_fence_y1, color='black') # label='Right Fence Segment 1'
# plt.plot(right_fence_x2, right_fence_y2, color='black') # label='Right Fence Segment 2'
# plt.plot(right_fence_x3, right_fence_y3, color='black') # label='Right Fence Segment 3'
# # Plotting the middle dashed line
# plt.axhline(0, color='black', linestyle='--', xmin=41/410, xmax=310/410)  # the xmin and xmax values are normalized