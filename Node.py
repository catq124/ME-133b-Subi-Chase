"""
Defines the wall spaces from a given map
in maps.py

Node class (to be used for catplanning and some human path)
checks distances, freespaces --> neighbors --> connections, 
line of sight connections. 

"""


from math import inf
import maps

map = maps.map # X and Y dims of the map

# Ask use to pickdifficulty
walls = maps.easy # Specific map walls

RWALL = []
UWALL = []
for wall in walls:
    for i in range(len(wall)-1):
        if wall[i][0] - wall[i+1][0] == 0: #horizontal
            for x in range(wall[i+1][1]-wall[i][1]-1):
                UWALL.append((wall[i][0], wall[i][1]+x))
        if wall[i][1] - wall[i+1][1] == 0: #vertical
            for y in range(wall[i+1][0] - wall[i][0]-1):
                RWALL.append((wall[i][0]+y, wall[i][1]))

dt = 0.5 #second increments
            
class Node:
    # Initialization
    def __init__(self, row, col, t):
        # Save the matching state.
        self.row = row
        self.col = col
        self.t = t

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
        self.caught = False

        # State motion freedom of node
        self.freeLeft = True
        self.freeRight = True
        self.freeUp = True
        self.freeDown = True
        
    # Checks degrees of motion
    def checkFree(self):
        coord = (self.row, self.col)
        if coord in RWALL:
            self.freeLeft = False
        
        if (self.row, self.col+1) in RWALL:
            self.freeRight = False

        if coord in UWALL:
            self.freeUp = False

        if (self.row+1, self.col) in UWALL:
            self.freeDown = False        

    # Define the Manhattan distance to another node.
    def distance(self, other):
        return abs(self.row - other.row) + abs(self.col - other.col)

    # Makes list of possible neighbors
    def findNeighbors(self):
        row = self.row
        col = self.col
        t = self.t + dt
        neighbors = []

        self.checkFree()

        if self.freeRight and col+1<map[1]:
            neighbors.append(Node(row, col+1, t))
        if self.freeLeft and col-1>=0:
            neighbors.append(Node(row, col-1, t))
        if self.freeUp and row-1>=0:
            neighbors.append(Node(row-1, col, t))
        if self.freeDown and row+1<map[0]:
            neighbors.append(Node(row+1, col, t))

        return neighbors

    # Check adjacent connection
    def connectsTo(self, other):
        neighbors = self.findNeighbors()
        if other in neighbors:
            return True
        return False
    
    # Returns orientation of line made by three points a, b, c (0: Straight, 1: Left, -1: Right)
    def orientation(a, b, c):
        # Cross product of line AB and line AC
        raw = (b[1]-a[1])*(c[0]-a[0]) - (b[0]-a[0])*(c[1]-a[1])
        return int(raw/abs(raw))
    
    # Check line of sight between two nodes (ie. connection)
    def lineOfSight(self, other): 
        node1 = (self.row, self.col)
        node2 = (other.row, other.col)
        for i in range(len(walls)):
            for j in range(len(walls[i])-1):
                wall1 = walls[i][j]
                wall2 = walls[i][j+1]
            if (self.orientation(node1, node2, wall1)*self.orientation(node1, node2, wall2) < 0 and
            self.orientation(wall1, wall2, node1)*self.orientation(wall1, wall2, node2) < 0):
                return False
        return True

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
