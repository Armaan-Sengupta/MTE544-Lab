
from mapUtilities import *
from a_star import *

POINT_PLANNER=0; TRAJECTORY_PLANNER=1

class planner:
    def __init__(self, type_, mapName="room", heuristic="euclidean", laser_sig=0.32, visualize=False):

        self.type=type_
        self.mapName=mapName
        self.heuristic=heuristic
        self.laser_sig=laser_sig
        self.visualize=visualize
        self.costMap=None

    
    def plan(self, startPose, endPose):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)
        
        elif self.type==TRAJECTORY_PLANNER:
            if self.costMap is None:
                self.initTrajectoryPlanner()
            return self.trajectory_planner(startPose, endPose)


    def point_planner(self, endPose):
        return endPose

    def initTrajectoryPlanner(self):


        # TODO PART 5 Create the cost-map, the laser_sig is 
        # the standard deviation for the gausiian for which
        # the mean is located on the occupant grid. 

        # Load the map found via slam
        # The default laser_sig of 0.32 accounts for the Tb4 dimensions and buffer distance.
        self.m_utilites = mapManipulator(filename_=self.mapName, laser_sig=self.laser_sig)

        # Create cost map of the map found via SLAM
        self.costMap = self.m_utilites.make_likelihood_field()
        

    def trajectory_planner(self, startPoseCart, endPoseCart):


        # This is to convert the cartesian coordinates into the 
        # the pixel coordinates of the map image, remmember,
        # the cost-map is in pixels. You can by the way, convert the pixels
        # to the cartesian coordinates and work by that index, the a_star finds
        # the path regardless. 
        startPose = self.m_utilites.position_2_cell(startPoseCart)
        endPose = self.m_utilites.position_2_cell(endPoseCart)
        
        # TODO PART 5 convert the cell pixels into the cartesian coordinates
        # Check both endpoints are within the map before indexing
        height, width = self.costMap.shape
        start_x, start_y = startPose
        end_x, end_y = endPose
        if not (0 <= start_x < width and 0 <= start_y < height):
            return None

        if not (0 <= end_x < width and 0 <= end_y < height):
            return None

        # Cost map indices are [y,x] with values above 0.8 being blocked
        if self.costMap[start_y, start_x] > 0.8:
            print(f"Start pose {startPoseCart} (cell {startPose}) is inside an obstacle")
            return None
        
        if self.costMap[end_y, end_x] > 0.8:
            print(f"Goal pose {endPoseCart} (cell {endPose}) is inside an obstacle")
            return None
        # Run A* on the cost map to get path in grid cell
        # Pass in the heuristic you want
        cell_path = search(
            self.costMap,
            startPose,
            endPose,
            heuristic=self.heuristic,
            visualize=self.visualize,
            raw_map=self.m_utilites.image_array,
        )

        # Reject any missing or partial paths that do not reach goal
        if not cell_path or cell_path[-1] != endPose:
            return None

        # TODO PART 5 return the path as list of [x,y]
        return [list(self.m_utilites.cell_2_position(cell)) for cell in cell_path]


if __name__=="__main__":

    m_utilites=mapManipulator()
    
    map_likelihood=m_utilites.make_likelihood_field()

    # you can use this part of the code to test your 
    # search algorithm regardless of the ros2 hassles
    
