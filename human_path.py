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
from maps import map, easy, med, diff
from maps import start_e, goal_e, start_m, goal_m, start_d, goal_d

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

######################################################################
#
#  Main Code
#
if __name__== "__main__":

    ####################  INITIALIZE  ####################
    # Choose the map.
    levels = {
        "1": (easy, start_e, goal_e),
        "2": (med, start_m, goal_m),
        "3": (diff, start_d, goal_d)
    }
    grid_choice = input("Choose level of difficulty (1 -> easy), (2 -> medium), (3 -> hard): ").strip()
    if grid_choice not in levels:
        raise ValueError("NOT A VALID INPUT")
    
    walls, start_xy, goal_xy = levels[grid_choice]

    # Grab the dimensions.
    rows = map[1]
    cols = map[0]

    # Set up the visual grid.
    visual = VisualGrid(rows, cols)

    # Parse the grid to set up the nodes list, as well as start/goal.
    blocked = wallpoints(walls)
    
    nodes  = []
    for y in range(1, rows + 1):
        for x in range(1, cols + 1):
            row, col = rows_cols(x, y, rows)

            # Create a node per space, except only color walls black.
            if (x, y) in blocked:
                visual.color(row, col, BLACK)
            else:
                nodes.append(Node(row, col))

    # Create the neighbors, being the edges between the nodes.
    for node in nodes:
        for (dr, dc) in [(-1,0), (1,0), (0,-1), (0,1)]:
            others = [n for n in nodes
                      if (n.row,n.col) == (node.row+dr,node.col+dc)]
            if len(others) > 0:
                node.neighbors.append(others[0])

    # Grab/mark the start/goal.
    srow, scol = rows_cols(start_xy[0], start_xy[1], rows)
    grow, gcol = rows_cols(goal_xy[0], goal_xy[1], rows)

    start = [n for n in nodes if (n.row, n.col) == (srow, scol)][0]
    goal  = [n for n in nodes if (n.row, n.col) == (grow, gcol)][0]

    visual.write(start.row, start.col, 'S')
    visual.write(goal.row,  goal.col,  'G')
    visual.show(wait="Hit return to start")
    ####################  RUN  ####################

    # Create a function to show each step.
    '''
    def show(wait=0.005):
        # Update the grid for all nodes.
        for node in nodes:
            # Choose the appropriate color.
            if   node.done: visual.color(node.row, node.col, BARK)
            elif node.seen: visual.color(node.row, node.col, GREEN)
            else:           visual.color(node.row, node.col, SKY)
        # Show.
        visual.show(wait)
    '''
    # Run.
    
    path = planner(start, goal)

    ####################  REPORT  ####################
    # Check the number of nodes.
    '''
    unknown   = len([n for n in nodes if not n.seen])
    processed = len([n for n in nodes if n.done])
    ondeck    = len(nodes) - unknown - processed
    print("Solution cost %f" % goal.cost)
    print("%3d states fully processed" % processed)
    print("%3d states still pending"   % ondeck)
    print("%3d states never reached"   % unknown)
    '''

    # Show the path in red.
    if not path:
        print("UNABLE TO FIND A PATH")
    else:
        print("Marking the human path")
        
        for i in range (len(path) - 1):
            
            r1 = path[i].row
            c1 = path[i].col
            r2 = path[i+1].row
            c2 = path[i+1].col

            visual.segment(r1,c1,r2,c2,RED)

        visual.show()

    input("Hit return to end")