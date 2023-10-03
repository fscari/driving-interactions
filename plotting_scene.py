import numpy as np
import matplotlib.pyplot as plt

def pltng_scene(x_positions_humancar, y_positions_humancar, x_positions_nestedcar, y_positions_nestedcar, theta, start_time, end_time):
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

    filename = '_'.join([str(round(t, 2)) for t in theta]) + '.png'
    plt.savefig(filename)