from pathlib import Path
import matplotlib.pyplot as plt
from utilities import FileReader




def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file()
    
    if not values:
        raise ValueError(f"No samples recorded in {filename}")

    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    
    
    fig, axes = plt.subplots(1,2, figsize=(14,6))


    axes[0].plot([lin[0] for lin in values], [lin[1] for lin in values])
    fig.suptitle(str(Path(filename)))
    axes[0].set_title("Robot path" if headers[0] == "x" else "Error phase plot")
    axes[0].set_xlabel("x [m]" if headers[0] == "x" else "Error [m or rad]")
    axes[0].set_ylabel("y [m]" if headers[0] == "x" else "Error derivative [m/s or rad/s]")
    axes[0].grid()

    
    axes[1].set_title("Recorded states")
    axes[1].set_xlabel("Elapsed time [s]")
    axes[1].set_ylabel("State value (see legend)")
    for i in range(0, len(headers) - 1):
        axes[1].plot(time_list, [lin[i] for lin in values], label=headers[i])

    axes[1].legend()
    axes[1].grid()

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



