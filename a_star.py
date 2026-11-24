import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from math import sqrt


class Node:
    """
        A node class for A* Pathfinding
        parent is parent of the current Node
        position is current position of the Node in the maze
        g is cost from start to current Node
        h is heuristic based estimated cost for current Node to end Node
        f is total cost of present node i.e. :  f = g + h
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

# This function return the path of the search


def return_path(current_node, maze):
    path = []
    no_rows, no_columns = np.shape(maze)
    # here we create the initialized result maze with -1 in every position
    result = [[-1 for i in range(no_columns)] for j in range(no_rows)]
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    # Return reversed path as we need to show from start to end path
    path = path[::-1]
    start_value = 0
    # we update the path of start to end found by A-star serch with every step incremented by 1
    for i in range(len(path)):
        result[path[i][0]][path[i][1]] = start_value
        start_value += 1

    return path


class _SearchVisualizer:
    """Live Matplotlib visualizer for A* search over occupancy/cost grids."""

    def __init__(self, cost_grid, start, end, heuristic, raw_map=None, threshold=0.8):
        self.heuristic = heuristic
        self.skip = False
        plt.ion()
        if raw_map is not None:
            self.fig, (self.ax_occ, self.ax_cost) = plt.subplots(1, 2, figsize=(13, 6))
            self.ax_occ.imshow(raw_map, cmap="gray", origin="upper", vmin=0, vmax=255)
            blocked = np.ma.masked_where(cost_grid <= threshold, cost_grid)
            self.ax_occ.imshow(blocked, cmap="autumn", origin="upper", alpha=0.45)
            self.ax_occ.set_title(f"Occupancy Grid + Blocked (> {threshold})")
            self.ax_occ.set_xlabel("cell x (col)")
            self.ax_occ.set_ylabel("cell y (row)")
            self.axes = (self.ax_occ, self.ax_cost)
        else:
            self.fig, self.ax_cost = plt.subplots(1, 1, figsize=(7, 6))
            self.ax_occ = None
            self.axes = (self.ax_cost,)

        self.ax_cost.imshow(cost_grid, cmap="viridis", origin="upper", vmin=0.0, vmax=1.0)
        self.ax_cost.set_xlabel("cell x (col)")
        self.ax_cost.set_ylabel("cell y (row)")

        for ax in self.axes:
            ax.plot(start[0], start[1], "go", markersize=8, label="Start")
            ax.plot(end[0], end[1], "r*", markersize=11, label="Goal")

        empty = np.empty((0, 2))
        self.visited_scatter = self.ax_cost.scatter(
            empty[:, 0], empty[:, 1], s=8, c="cyan", alpha=0.5, label="Visited"
        )
        self.frontier_scatter = self.ax_cost.scatter(
            empty[:, 0], empty[:, 1], s=10, c="yellow", alpha=0.8, label="Frontier"
        )
        self.ax_cost.legend(loc="upper right", fontsize=8)
        self.ax_cost.set_title(f"A* ({heuristic}) | starting...")
        self.fig.tight_layout(rect=[0, 0.08, 1, 1])
        ax_btn = self.fig.add_axes([0.80, 0.015, 0.16, 0.045])
        self.skip_button = Button(ax_btn, "Skip Animation")
        self.skip_button.on_clicked(self._on_skip)
        plt.pause(0.001)

    def _on_skip(self, _event):
        self.skip = True

    def update(self, iterations, visited_dict, yet_to_visit_dict):
        if self.skip:
            return
        if visited_dict:
            self.visited_scatter.set_offsets(np.array(list(visited_dict.keys())))
        if yet_to_visit_dict:
            self.frontier_scatter.set_offsets(np.array(list(yet_to_visit_dict.keys())))
        self.ax_cost.set_title(
            f"A* ({self.heuristic}) | iter={iterations} visited={len(visited_dict)} open={len(yet_to_visit_dict)}"
        )
        self.fig.canvas.draw_idle()
        plt.pause(0.001)

    def finish(self, path, visited_dict, yet_to_visit_dict, iterations, block=True):
        self.skip_button.ax.set_visible(False)
        if visited_dict:
            self.visited_scatter.set_offsets(np.array(list(visited_dict.keys())))
        if yet_to_visit_dict:
            self.frontier_scatter.set_offsets(np.array(list(yet_to_visit_dict.keys())))
        if path:
            px, py = zip(*path)
            for ax in self.axes:
                ax.plot(px, py, color="lime", linewidth=2.0, label="Path")
                ax.legend(loc="upper right", fontsize=8)
        self.ax_cost.set_title(
            f"A* ({self.heuristic}) | iter={iterations} visited={len(visited_dict)} path={len(path) if path else 0} (close window to start)"
        )
        self.fig.canvas.draw_idle()
        plt.ioff()
        if block:
            plt.show()
        else:
            plt.show(block=False)
            plt.pause(0.1)


def search(maze, start, end, heuristic="euclidean", visualize=False, raw_map=None, threshold=0.8, block=True):

    print("searching ....")

    maze = maze.T

    """
        Returns a list of tuples as a path from the given start to the given end in the given maze
        :param maze:
        :param cost
        :param start:
        :param end:
        :param heuristic: "euclidean" or "manhattan"
        :return: Array of positions (tuples)
    """    
    # Choose huristic default being euclidean
    if heuristic not in ("euclidean", "manhattan"):
        raise ValueError("heuristic must be 'euclidean' or 'manhattan'")

    # Distance calculation between start and end nodes for either heuristic
    def distance_to_goal(position):
        dx = abs(position[0] - end[0])
        dy = abs(position[1] - end[1])
        if heuristic == "manhattan":
            return (dx + dy)
        elif heuristic == "euclidean":
            return sqrt(dx**2 + dy**2)
        else:
            return 0

    # TODO PART 4 Create start and end node with initized values for g, h and f
    # Use None as parent if not defined

    # g is current cost so far, h is estimated cost to the goal
    # f = g + h determines which node to expand to next
    start_node = Node(None, tuple(start))
    start_node.g = 0    # cost from start Node
    start_node.h = distance_to_goal(start_node.position)     # heuristic estimated cost to end Node
    start_node.f = start_node.g + start_node.h # Current cost + Heuristic cost

    # Search uses end node's position to detect when it reaches the goal
    end_node = Node(None, tuple(end))
    end_node.g = float("inf")      # set a large value if not defined
    end_node.h = distance_to_goal(end_node.position)       # heuristic estimated cost to end Node
    end_node.f = end_node.g + end_node.h # Current cost + Heuristic cost

    # Initialize both yet_to_visit and visited dictionary
    # in this dict we will put all node that are yet_to_visit for exploration.
    # From here we will find the lowest cost node to expand next
    yet_to_visit_dict = {}  # key is the position (tuple), value is the node
    # in this list we will put all node those already explored so that we don't explore it again
    # key is the position (tuple), value is True (boolean)
    visited_dict = {}

    # Add the start node
    yet_to_visit_dict[start_node.position] = start_node

    # Adding a stop condition. This is to avoid any infinite loop and stop
    # execution after some reasonable number of steps
    outer_iterations = 0
    max_iterations = (len(maze) // 2) ** 10

    # TODO PART 4 what squares do we search . serarch movement is left-right-top-bottom
    # (4 or 8 movements) from every positon
    move = [[0, -1],  # go up
            [-1, 0],  # go left
            [0, 1],   # go down
            [1, 0]]  # go right

    # To ensure A* finds optimal path only allow Manhattan to move in 4pt connectivity
    # This ensures herustic costy is an under-estimate of the actual cost-to-go
    if heuristic == "euclidean":
        move.extend([[-1, -1],  # go up left
                     [-1, 1],   # go down left
                     [1, -1],   # go up right
                     [1, 1]])   # go down right

    """
        1) We first get the current node by comparing all f cost and selecting the lowest cost node for further expansion
        2) Check max iteration reached or not . Set a message and stop execution
        3) Remove the selected node from yet_to_visit dict and add this node to visited dict
        4) Perofmr Goal test and return the path else perform below steps
        5) For selected node find out all children (use move to find children)
            a) get the current postion for the selected node (this becomes parent node for the children)
            b) check if a valid position exist (boundary will make few nodes invalid)
            c) if any node is a wall then ignore that
            d) add to valid children node list for the selected parent
            
            For all the children node
                a) if child in visited dict then ignore it and try next node
                b) calculate child node g, h and f values
                c) if child in yet_to_visit dict then ignore it
                d) else move the child to yet_to_visit dict
    """
    # TODO PART 4 find maze has got how many rows and columns
    no_rows, no_columns = maze.shape

    # Loop until you find the end
    viz = _SearchVisualizer(maze.T, start, end, heuristic, raw_map=raw_map, threshold=threshold) if visualize else None

    while len(yet_to_visit_dict) > 0:

        # Every time any node is referred from yet_to_visit list, counter of limit operation incremented
        outer_iterations += 1

        # Get the current node with the lowest f value
        current_node = None
        current_fscore = None
        for position, node in yet_to_visit_dict.items():
            if current_fscore is None or node.f < current_fscore:
                current_fscore = node.f
                current_node = node

        # if we hit this point return the path such as it may be no solution or
        # computation cost is too high
        if outer_iterations > max_iterations:
            print("giving up on pathfinding too many iterations")
            path = return_path(current_node, maze)
            if viz is not None:
                viz.finish(path, visited_dict, yet_to_visit_dict, outer_iterations, block=block)
            return path

        # Pop current node out off yet_to_visit dict, add to visited list
        yet_to_visit_dict.pop(current_node.position)
        visited_dict[current_node.position] = True

        if viz is not None and outer_iterations % 25 == 0:
            viz.update(outer_iterations, visited_dict, yet_to_visit_dict)

        # test if goal is reached or not, if yes then return the path
        if current_node == end_node:

            path = return_path(current_node, maze)
            if viz is not None:
                viz.finish(path, visited_dict, yet_to_visit_dict, outer_iterations, block=block)
            return path

        # Generate children from all adjacent squares
        children = []

        for new_position in move:

            # TODO PART 4 Get node position
            node_position = (current_node.position[0]+ new_position[0],
                             current_node.position[1] + new_position[1])

            # TODO PART 4 Make sure within range (check if within maze boundary)
            if (node_position[0] < 0 or node_position[0] >= no_rows or
                node_position[1] < 0 or node_position[1] >= no_columns):
                continue

            # Make sure walkable terrain
            if maze[node_position[0], node_position[1]] > 0.8:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children

        for child in children:

            # TODO PART 4 Child is on the visited dict (use get method to check if child is in visited dict, if not found then default value is False)
            if (visited_dict.get(child.position, False)):
                continue

            # TODO PART 4 Create the f, g, and h values
            # Cost to get to current cell plus one more step
            child.g = current_node.g + 1
            # Heuristic costs calculated here, this is using eucledian distance
            child.h = distance_to_goal(child.position)
            # Total Expected Cost
            child.f = child.g + child.h

            # Child is already in the yet_to_visit list and g cost is already lower
            child_node_in_yet_to_visit = yet_to_visit_dict.get(
                child.position, False)
            if (child_node_in_yet_to_visit is not False) and (child.g >= child_node_in_yet_to_visit.g):
                continue

            # Add the child to the yet_to_visit list
            yet_to_visit_dict[child.position] = child
