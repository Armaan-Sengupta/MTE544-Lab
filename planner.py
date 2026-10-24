from math import exp

# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1



class planner:
    def __init__(self, type_, trajectory="parabola"):

        self.type=type_
        self.trajectory=trajectory

    
    def plan(self, goalPoint=[-1.0, -1.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self):
        # Sample at 0.05 m spacing in x, including both endpoints.
        # Return a list of trajectory points: [[x1,y1], ..., [xn,yn]].
        if self.trajectory == "parabola":
            return [[i * 0.05, (i * 0.05)**2] for i in range(31)]
        elif self.trajectory == "sigmoid":
            return [[i * 0.05, 2 / (1 + exp(-2 * i * 0.05)) - 1]
                    for i in range(51)]
        raise ValueError(f"Unknown trajectory: {self.trajectory}")

