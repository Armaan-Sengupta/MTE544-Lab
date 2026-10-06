# You can use this file to plot the loged sensor data
# Note that you need to modify/adapt it to your own files
# Feel free to make any modifications/additions here

import matplotlib.pyplot as plt
import numpy as np
from utilities import FileReader

def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file() 
    time_list=[]
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    for i in range(0, len(headers) - 1):
        plt.plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")
    
    #plt.plot([lin[0] for lin in values], [lin[1] for lin in values])
    plt.legend()
    plt.grid()
    plt.show()
    
def plot_laser(filename):
    with open(filename) as file:
        scans = list(file)[1:] 

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
            if delta_time > 1e4:  # convert nanosecondss to seconds
                delta_time /= 1e9
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
        plt.scatter(x, y, s=10)
        plt.title(filename)
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.axis("equal")
        plt.grid()
        plt.draw()

    if plt.fignum_exists(fig.number):
        plt.show()

import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        if filename in ("laser_content_line.csv", "laser_content_circle.csv", "laser_content_spiral.csv"):
            plot_laser(filename)
        else:
            plot_errors(filename)
