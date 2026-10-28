import json
from math import atan2, cos, sin, sqrt
from pathlib import Path
import matplotlib.pyplot as plt
from utilities import FileReader


def planned_path(filename):
    """Return the sampled trajectory from the run.json file, if this is a path run."""
    run_file = Path(filename).with_name("run.json")
    if not run_file.is_file():
        return None, None

    with open(run_file) as file:
        metadata = json.load(file)

    goal = metadata.get("goal")
    # Point runs store [x, y]. Trajectory runs store [[x, y], ...].
    if not goal or not isinstance(goal[0], (list, tuple)):
        return None, None
    return goal, metadata.get("trajectory")


def pose_velocities(filename):
    """Return elapsed pose time, linear speed, and rotational velocity."""
    pose_file = Path(filename).with_name("robot_pose.csv")
    if not pose_file.is_file():
        return None

    headers, values = FileReader(pose_file).read_file()
    if not values:
        raise ValueError(f"No pose samples recorded in {pose_file}")
    required_headers = ("x", "y", "theta", "stamp")
    if tuple(headers) != required_headers:
        raise ValueError(
            f"Expected {required_headers} in {pose_file}, found {tuple(headers)}"
        )

    first_stamp = values[0][3]
    elapsed = [row[3] - first_stamp for row in values]
    linear_speed = [0.0]
    rotational_velocity = [0.0]

    for previous, current in zip(values, values[1:]):
        dt = current[3] - previous[3]
        if dt <= 0:
            raise ValueError(f"Pose timestamps must increase in {pose_file}")

        distance = sqrt(
            (current[0] - previous[0]) ** 2
            + (current[1] - previous[1]) ** 2
        )
        heading_delta = atan2(
            sin(current[2] - previous[2]),
            cos(current[2] - previous[2]),
        )
        linear_speed.append(distance / dt)
        rotational_velocity.append(heading_delta / dt)

    return elapsed, linear_speed, rotational_velocity


def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file()
    
    if not values:
        raise ValueError(f"No samples recorded in {filename}")

    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    
    
    velocities = pose_velocities(filename)
    figure_count = 3 if velocities else 2
    fig, axes = plt.subplots(1, figure_count, figsize=(7 * figure_count, 6))


    is_path = headers[0] == "x"
    axes[0].plot(
        [lin[0] for lin in values],
        [lin[1] for lin in values],
        label="actual" if is_path else None,
    )
    if is_path:
        path, name = planned_path(filename)
        if path:
            label = f"ideal ({name})" if name else "ideal"
            axes[0].plot([point[0] for point in path], [point[1] for point in path], "--", label=label)
        axes[0].legend()
        axes[0].set_aspect("equal", adjustable="datalim")
    fig.suptitle(str(Path(filename)))
    axes[0].set_title("Robot path" if is_path else "Error phase plot")
    axes[0].set_xlabel("x [m]" if is_path else "Error [m or rad]")
    axes[0].set_ylabel("y [m]" if is_path else "Error derivative [m/s or rad/s]")
    axes[0].grid()

    
    axes[1].set_title("Recorded states")
    axes[1].set_xlabel("Elapsed time [s]")
    axes[1].set_ylabel("State value (see legend)")
    for i in range(0, len(headers) - 1):
        axes[1].plot(time_list, [lin[i] for lin in values], label=headers[i])

    axes[1].legend()
    axes[1].grid()

    if velocities:
        velocity_time, linear_speed, rotational_velocity = velocities
        axes[2].plot(velocity_time, linear_speed, label="linear speed [m/s]")
        axes[2].plot(
            velocity_time,
            rotational_velocity,
            label="rotational velocity [rad/s]",
        )
        axes[2].set_title("Estimated velocities from robot pose")
        axes[2].set_xlabel("Elapsed time [s]")
        axes[2].set_ylabel("Velocity")
        axes[2].legend()
        axes[2].grid()

    plt.show()
    
    





import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        plot_errors(filename)


