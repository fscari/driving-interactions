import os
import matplotlib.pyplot as plt


def pltng_scene(vehicles_data, theta, session, vehicle_nr, condition_name, experiment_nr, count, waypoints_x_locations=0, waypoints_y_locations=0, used_wayponts_x_location=0, used_wayponts_y_location=0):
    folder_name = "Experiment Nr_" + str(experiment_nr)
    if not os.path.exists(folder_name):
        os.mkdir(folder_name)
    title = "Experiment Nr_" + str(experiment_nr) + " _Session_" + str(session) + "_Vehicle_NR_" + str(vehicle_nr) + "_Condition_" + condition_name + "_Iteration Nr_" + str(count)
    file_path_csv = os.path.join(folder_name, title + '.csv')
    file_path_plot = os.path.join(folder_name, title + '.png')
    # Create a figure with two subplots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))  # 1 row, 2 columns
    # Left fence coordinates
    left_fence_x = [50, 800]
    left_fence_y = [-3.5, -3.5]
    # Right fence coordinates
    right_fence_x0 = [50, 365]
    right_fence_y0 = [19.2, 3.5]
    right_fence_x1 = [365, 550]
    right_fence_y1 = [3.5, 3.5]
    right_fence_x2 = [550, 600]
    right_fence_y2 = [3.5, 0]
    right_fence_x3 = [600, 800]
    right_fence_y3 = [0, 0]
    middle_fence_l_x = [50, 365]
    middle_fence_l_y = [0, 0]
    middle_fence_r_x = [50, 365]
    middle_fence_r_y = [15.5, 0]
    # Plotting
    ax1.plot(left_fence_x, left_fence_y, color='black') # label='Left Fence')
    ax1.plot(right_fence_x0, right_fence_y0, color='black')  # label='Right Fence Segment 0'
    ax1.plot(right_fence_x1, right_fence_y1, color='black') # label='Right Fence Segment 1'
    ax1.plot(right_fence_x2, right_fence_y2, color='black') # label='Right Fence Segment 2'
    ax1.plot(right_fence_x3, right_fence_y3, color='black') # label='Right Fence Segment 3'
    ax1.plot(middle_fence_l_x, middle_fence_l_y, color='black') # label='Middle Fence Left'
    ax1.plot(middle_fence_r_x, middle_fence_r_y, color='black')  # label='Middle Fence Right'
    # Plotting the middle dashed line
    ax1.axhline(0, color='black', linestyle='--', xmin=41/410, xmax=310/410)  # the xmin and xmax values are normalized
    x_positions_humancar = vehicles_data['x_positions_human_car'].tolist()
    y_positions_humancar = vehicles_data['y_positions_human_car'].tolist()
    filtered_x_positions_humancar = []
    filtered_y_positions_humancar = []
    for x_val, y_val in zip(x_positions_humancar, y_positions_humancar):
        if x_val != 0 or y_val != 0:
            filtered_x_positions_humancar.append(x_val)
            filtered_y_positions_humancar.append(y_val)

    x_positions_nestedcar = vehicles_data['x_positions_nested_car'].tolist()
    y_positions_nestedcar = vehicles_data['y_positions_nested_car'].tolist()
    filtered_x_positions_nestedcar = []
    filtered_y_positions_nestedcar = []
    for x_val, y_val in zip(x_positions_nestedcar, y_positions_nestedcar):
        if x_val != 0 or y_val != 0:
            filtered_x_positions_nestedcar.append(x_val)
            filtered_y_positions_nestedcar.append(y_val)

    ax1.plot(filtered_x_positions_nestedcar[:], filtered_y_positions_nestedcar[:], color='green', label='nestedcar',
             linewidth=2)
    ax1.plot(filtered_x_positions_humancar[:], filtered_y_positions_humancar[:], color='red', label='humancar',
             linewidth=2)
    if waypoints_x_locations != 0 and waypoints_y_locations != 0:
        ax1.plot(waypoints_x_locations[:], waypoints_y_locations[:], color='orange', label='waypoints',
             linewidth=1)
    # Additional plot settings
    ax1.set_xlabel('X Position')
    ax1.set_ylabel('Y Position')
    ax1.set_title(theta)
    ax1.grid(False)
    ax1.invert_yaxis()
    ax1.legend()

    left_fence_x1 = [360, 800]

# Scatterplot for velocities
    ax2.plot(left_fence_x1, left_fence_y, color='black')  # label='Left Fence')
    ax2.plot(right_fence_x1, right_fence_y1, color='black')  # label='Right Fence Segment 1'
    ax2.plot(right_fence_x2, right_fence_y2, color='black')  # label='Right Fence Segment 2'
    ax2.plot(right_fence_x3, right_fence_y3, color='black')  # label='Right Fence Segment 3'

    nested_closest_position = min(filtered_x_positions_nestedcar, key=lambda x: abs(x - 363.0))
    nested_index_of_367 = filtered_x_positions_nestedcar.index(nested_closest_position)
    filtered_x_positions_nestedcar1 = filtered_x_positions_nestedcar[nested_index_of_367:]
    filtered_y_positions_nestedcar1 = filtered_y_positions_nestedcar[nested_index_of_367:]

    human_closest_position = min(filtered_x_positions_humancar, key=lambda x: abs(x - 363.0))
    human_index_of_367 = filtered_x_positions_humancar.index(human_closest_position)
    filtered_x_positions_humancar1 = filtered_x_positions_humancar[human_index_of_367:]
    filtered_y_positions_humancar1 = filtered_y_positions_humancar[human_index_of_367:]
    # Plotting the middle dashed line
    ax2.axhline(0, color='black', linestyle='--', xmin=41 / 410,
                xmax=310 / 410)  # the xmin and xmax values are normalized

    ax2.scatter(filtered_x_positions_nestedcar1[::2], filtered_y_positions_nestedcar1[::2], color='green',
                label='nestedcar', s=5)
    ax2.scatter(filtered_x_positions_humancar1[::2], filtered_y_positions_humancar1[::2], color='red',
                label='nestedcar', s=5)

    # Additional plot settings
    ax2.set_xlabel('X Position')
    ax2.set_ylabel('Y Position')
    ax2.set_title(theta)
    ax2.grid(False)
    ax2.invert_yaxis()
    ax2.legend()
    plt.savefig(file_path_plot)
    # plt.show()

    if used_wayponts_x_location != 0 and used_wayponts_y_location != 0:
        y_error = [a - b for a, b in zip(used_wayponts_y_location, filtered_y_positions_nestedcar)]
        plt.plot(y_error)
        plt.show()
        print(y_error)
        # plt.plot(y_error, used_wayponts_y_location)
        # plt.show()
    vehicles_data.to_csv(file_path_csv, index=False)
