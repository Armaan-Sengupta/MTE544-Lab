# You can use this file to plot the loged sensor data
# Note that you need to modify/adapt it to your own files
# Feel free to make any modifications/additions here

import os
import matplotlib.pyplot as plt
import numpy as np
from utilities import FileReader

def plot_errors(filename, title=None, y_label=None, save_suffix="", show=True):
    """Shared time-series plotting logic for sensor data vs time."""
    headers, values = FileReader(filename).read_file()
    if not values:
        return

    os.makedirs('plots', exist_ok=True)
    base_name = os.path.splitext(os.path.basename(filename))[0] #odom, imu, or laser

    # Convert timestamps from nanoseconds to seconds of elpased time
    first_stamp = values[0][-1]
    time_list = [(val[-1] - first_stamp) / 1e9 for val in values]

    markers = ['o', 's', '^', 'D', 'v', '<', '>']
    colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown']
    markevery = max(1, len(values) // 25)

    plt.figure()
    for i in range(0, len(headers) - 1):
        plt.plot(
            time_list,
            [lin[i] for lin in values],
            label=headers[i],
            color=colors[i % len(colors)],
            marker=markers[i % len(markers)],
            markevery=markevery
        )

    plt.title(title or f"Sensor / Error Data vs Time - {filename}")
    plt.xlabel("Time (s)")
    plt.ylabel(y_label or "Value")
    plt.legend()
    plt.grid(True)
    plt.savefig(f"plots/{base_name}{save_suffix}.png", bbox_inches='tight')
    if show:
        plt.show()

def plot_odom(filename):
    """Plot odometry: 1. x vs y. 2. (x, y, th vs t)."""
    headers, values = FileReader(filename).read_file()
    if not values:
        return

    os.makedirs('plots', exist_ok=True)
    base_name = os.path.splitext(os.path.basename(filename))[0]
    markevery = max(1, len(values) // 25)

    # 1. Cartesian trajectory (x vs y)
    x_vals = [lin[0] for lin in values]
    y_vals = [lin[1] for lin in values]

    plt.figure()
    plt.plot(x_vals, y_vals, label='Trajectory path', color='tab:blue', linestyle='-', marker='.', markevery=markevery)
    plt.scatter([x_vals[0]], [y_vals[0]], color='tab:green', marker='o', s=100, label='Start Point', zorder=5)
    plt.scatter([x_vals[-1]], [y_vals[-1]], color='tab:red', marker='s', s=100, label='End Point', zorder=5)
    plt.title(f"Odometry Trajectory (x vs y) - {filename}")
    plt.xlabel("x Position (m)")
    plt.ylabel("y Position (m)")
    plt.axis("equal")
    plt.legend()
    plt.grid(True)
    plt.savefig(f"plots/{base_name}_trajectory.png", bbox_inches='tight')

    # 2. States vs time (x,y,theta vs t)
    plot_errors(
        filename,
        title=f"Odometry States vs Time (x, y, θ vs t) - {filename}",
        y_label="Position (m) / Orientation (rad)",
        save_suffix="_states",
        show=False
    )
    plt.show()

def plot_imu(filename):
    """Plot IMU data: linear accelerations and angular velocity vs time."""
    plot_errors(
        filename,
        title=f"IMU Sensor Data vs Time ($a_x$, $a_y$, $\\omega_z$ vs t) - {filename}",
        y_label="Linear Acc (m/s^2) / Angular Vel (rad/s)"
    )

def plot_laser(filename):
    """Plot laser scan Cartesian point cloud animation and save the final frame."""
    with open(filename) as file:
        scans = list(file)[1:] 

    if not scans:
        return

    os.makedirs('plots', exist_ok=True)
    base_name = os.path.splitext(os.path.basename(filename))[0] #odom, laser, or imu

    fig = plt.figure()
    prev_time = None

    for scan in scans:
        if not plt.fignum_exists(fig.number):
            break

        values = scan.strip().split(',')
        if len(values) < 3 or not values[0].strip():
            continue

        ranges = np.array([float(value) for value in values[0].split()])
        angle_increment = float(values[1])
        time = float(values[2])

        if prev_time is not None:
            delta_time = time - prev_time
            delta_time /= 1e9 #nano seconds to seconds
            if delta_time > 0:
                plt.pause(delta_time)

        prev_time = time

        finite = np.isfinite(ranges)
        max_range = ranges[finite].max() if np.any(finite) else 0.0
        ranges[~finite] = max_range

        angles = np.arange(len(ranges)) * angle_increment
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        plt.cla()
        plt.scatter(x, y, s=12, c='tab:blue', marker='o', label='Scan points')
        plt.scatter([0], [0], s=80, c='tab:red', marker='^', label='Robot pose')
        plt.title(f"Laser Scan - {filename}")
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.axis("equal")
        plt.legend(loc='upper right')
        plt.grid(True)
        plt.draw()

    plt.savefig(f"plots/{base_name}.png", bbox_inches='tight')
    if plt.fignum_exists(fig.number):
        plt.show()

import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames = args.files
    for filename in filenames:
        if "laser" in filename.lower():
            plot_laser(filename)
        elif "odom" in filename.lower():
            plot_odom(filename)
        elif "imu" in filename.lower():
            plot_imu(filename)
        else:
            plot_errors(filename)
