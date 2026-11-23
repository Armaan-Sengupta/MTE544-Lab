
from pathlib import Path
from pid import PID_ctrl

import numpy as np

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error

M_PI=3.1415926535

P=0; PD=1; PI=2; PID=3

class controller:
    
    
    
    def __init__(self, klp=0.4, klv=0.08, kli=0.15, kap=1, kav=0.214, kai=0.064, control_type=PID, log_dir="."):
        
        self.PID_linear=PID_ctrl(control_type, klp, klv, kli, filename_=Path(log_dir) / "linear.csv")
        self.PID_angular=PID_ctrl(control_type, kap, kav, kai, filename_=Path(log_dir) / "angular.csv")

    
    def vel_request(self, pose, goal, status):
        
        e_lin=calculate_linear_error(pose, goal)
        e_ang=calculate_angular_error(pose, goal)

        
        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status) 

        linear_vel = 0.31 if linear_vel > 0.31 else linear_vel
        linear_vel = -0.31 if linear_vel < -0.31 else linear_vel
        angular_vel = 1.9 if angular_vel > 1.9 else angular_vel
        angular_vel = -1.9 if angular_vel < -1.9 else angular_vel


        return linear_vel, angular_vel
    

class trajectoryController(controller):

    def __init__(self, klp=0.05, klv=0.058, kli=0.061, kap=0.5, kav=0.214, kai=0.064, control_type=PID, log_dir=".", lookAhead=1.0, targetVel=1.0):
        super().__init__(klp, klv, kli, kap, kav, kai, control_type, log_dir)
        self.lookAhead=lookAhead
        self.targetVelocity=targetVel
    
    def vel_request(self, pose, listGoals, status):
        
        goal=self.lookFarFor(pose, listGoals)
        
        finalGoal=listGoals[-1]
        
  
        e_lin=calculate_linear_error(pose, finalGoal)
        e_ang=calculate_angular_error(pose, goal)

        
        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status) 

        linear_vel *= max(0.0, 1 - abs(e_ang)/0.5)
        linear_vel = max(0.0, min(0.22, linear_vel))
        angular_vel = max(-1.9, min(1.9, angular_vel))


        return linear_vel, angular_vel


    def lookFarFor(self, pose, listGoals):
        
        poseArray=np.array([pose[0], pose[1]]) 
        listGoalsArray=np.array(listGoals)

        distanceSquared=np.sum((listGoalsArray-poseArray)**2,
                               axis=1)
        closestIndex=np.argmin(distanceSquared)

        return listGoals[ min(closestIndex + 3, len(listGoals) - 1) ]