import csv
from math import atan2, asin, sqrt

M_PI=3.1415926535

class Logger:
    """Write each sample immediately so interrupted experiments retain their data."""

    def __init__(self, filename, headers=("e", "e_dot", "e_int", "stamp")):
        self.filename = filename
        with open(self.filename, 'w', newline='') as file:
            csv.writer(file).writerow(headers)

    def log_values(self, values_list):
        with open(self.filename, 'a', newline='') as file:
            csv.writer(file).writerow(values_list)

    def save_log(self):
        # Each log_values call already closes and flushes its file.
        pass

class FileReader:
    def __init__(self, filename):
        
        self.filename = filename
        
        
    def read_file(self):
        
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:

            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                
                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table
    
    

# TODO Part 3: Implement the conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    x, y, z, w = quat
    yaw = atan2(2*(w*z + x*y), 1-2*(y*y + z*z))
    return yaw


#TODO Part 4: Implement the calculation of the linear error
def calculate_linear_error(current_pose, goal_pose):
        
    # Compute the linear error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y]
    # Remember to use the Euclidean distance to calculate the error.
    cur_x, cur_y = current_pose[0], current_pose[1]
    goal_x, goal_y = goal_pose[0], goal_pose[1]
    error_linear = ((cur_x - goal_x)**2 + (cur_y - goal_y)**2)**(1/2)

    return error_linear

#TODO Part 4: Implement the calculation of the angular error
def calculate_angular_error(current_pose, goal_pose):

    # Compute the linear error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y]
    cur_x, cur_y = current_pose[0], current_pose[1]
    goal_x, goal_y = goal_pose[0], goal_pose[1]
    
    # Use atan2 to find the desired orientation
    # Remember that this function returns the difference in orientation between where the robot currently faces and where it should face to reach the goal

    error_angular = atan2((goal_y - cur_y), (goal_x - cur_x)) - current_pose[2]

    # Remember to handle the cases where the angular error might exceed the range [-π, π]

    if error_angular > M_PI:
        error_angular -= 2*M_PI
    elif error_angular < -M_PI:
        error_angular += 2*M_PI
    return error_angular
