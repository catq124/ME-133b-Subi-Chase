'''

   human_path.py

   This is the skeleton code for the grid planner, which will develop
   both Dijstra's and the Astar algorithms.

   When working, this will find a path in a simple 2D grid.

   This will eventually become our human class.

'''

import bisect

from math       import inf
from visualgrid import VisualGrid
import maps
import matplotlib.pyplot as plt
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import Node as time_node

# defining the grid

def wallpoints(walls):

    # use set to avoid duplicate points at corners
    points = set()
    for wall in walls:
        for i in range(len(wall) - 1):
            (x1, y1) = wall[i]
            (x2, y2) = wall[i+1]
            # if vertical wall
            if x1 == x2:
                ymin = min(y1, y2)
                ymax = max(y1, y2)
                for y in range(ymin, ymax+1):
                    points.add((x1, y))
            # if horizontal wall
            elif y1 == y2:
                xmin = min(x1, x2)
                xmax = max(x1, x2)
                for x in range(xmin, xmax+1):
                    points.add((x, y1))

    return points

def rows_cols(x, y, rows):
    
    row = rows - y
    col = x - 1 
    
    return (row, col)

#
#   Colors
#

WHITE  = [1.000, 1.000, 1.000]
BLACK  = [0.000, 0.000, 0.000]
RED    = [1.000, 0.000, 0.000]
BARK   = [0.325, 0.192, 0.094]  # bark brown
GREEN  = [0.133, 0.545, 0.133]  # forrest green
SKY    = [0.816, 0.925, 0.992]  # light blue

#
#   Node Class
#
#   We create one node to match each valid state being the robot
#   occupying a valid space in the grid.  That is, one node per
#   unblocked grid element (with a row/column).
#
#   To encode the graph, we also note a list of accessible neighbors.
#   And, as part of the search, we store the parent (in the tree), the
#   cost to reach this node (via the tree), and the status flags.
#
class Node:
    # Initialization
    def __init__(self, row, col):
        # Save the matching state.
        self.row = row
        self.col = col

        # Clear the list of neighbors (used for the full graph).
        self.neighbors = []

        # Clear the parent (used for the search tree), as well as the
        # actual cost to reach (via the parent).
        # You may want to add further information as needed here.
        self.parent = None      # No parent
        self.cost   = inf       # Unable to reach = infinite cost
        self.c_reach = inf

        # State of the node during the search algorithm.
        self.seen = False
        self.done = False


    # Define the Manhattan distance to another node.
    def distance(self, other):
        return abs(self.row - other.row) + abs(self.col - other.col)

    # Define the "less-than" to enable sorting by cost.
    def __lt__(self, other):
        return self.cost < other.cost

    # Print (for debugging).
    def __str__(self):
        return("(%2d,%2d)" % (self.row, self.col))
    
    def __repr__(self):
        return("<Node %s, %7s, cost %f>" %
               (str(self),
                "done" if self.done else "seen" if self.seen else "unknown",
                self.cost))

#
#   Search/Planner Algorithm
#
#   This is the core algorithm.  It builds a search tree inside the
#   node graph, transfering nodes from air (not seen) to leaf (seen,
#   but not done) to trunk (done).
#
# Run the planner.
def planner(start, goal, show = None, a=1):
    # Use the start node to initialize the on-deck queue: it has no
    # parent (being the start), zero cost to reach, and has been seen.
    start.seen   = True
    start.cost = 0
    start.parent = None
    onDeck = [start]

    # Continually expand/build the search tree.
    print("Starting the processing...")
    while True:
        # Show the grid.
        if show:
            show()

        # Make sure we have something pending in the on-deck queue.
        # Otherwise we were unable to find a path!
        if not (len(onDeck) > 0):
            return None

        # Grab the next state (first on the storted on-deck list).
        node = onDeck.pop(0)
        
        # Mark as done and check if the goal is thereby done.
        node.done = True
        if goal.done:
            break

        # Check the neighbors
        for neighbor in node.neighbors:
            # Skip if already fully proocessed.
            if neighbor.done:
                continue

            # If already seen (thereby on deck), we can also skip
            if neighbor.seen:
                continue
            
            # Save this new path and append to OnDeck
            neighbor.seen = True
            neighbor.cost = node.cost + 1
            neighbor.parent = node
            onDeck.append(neighbor)

    # Create the path to the goal (backwards) and show
    path = [goal]
    while path[0].parent:
        path.insert(0, path[0].parent)
    return path

def time(path, dt = 0.05):
    time_path = []
    t = 0
    for node in path:
        time_path.append(time_node.Node(node.row, node.col, t))
        t += dt
    
    return time_path

    human = AnnotationBbox(
        imagebox,
        (start.col + 0.5, rows - start.row - 0.5),
        frameon=False,
        zorder=5)
    
    visual.ax.add_artist(human)

    visual.show(wait="Hit return to start")
  
####################  REPORT  ####################

    # Show the path in red.
    if not path:
        print("UNABLE TO FIND A PATH")
    else:
        print("Marking the human path")

        # draw the entire path and keep it
        for i in range(len(path) - 1):
            r1 = path[i].row
            c1 = path[i].col
            r2 = path[i+1].row
            c2 = path[i+1].col

            visual.ax.plot(
                [c1 + 0.5, c2 + 0.5],
                [rows - r1 - 0.5, rows - r2 - 0.5],
                color='red',
                linewidth=3,
                zorder=4
            )

        for i in range(1, len(path)):
            human.remove()

            human = AnnotationBbox(
                imagebox,
                (path[i].col + 0.5, rows - path[i].row - 0.5),
                frameon=False,
                zorder=5
            )
            visual.ax.add_artist(human)

            visual.show(0.25)

    input("Hit return to end")