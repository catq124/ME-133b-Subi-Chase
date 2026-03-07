from visualgrid import VisualGrid
import matplotlib.pyplot as plt
from math import inf
import maps
import Node
import catplanner


if __name__== "__main__":

    map = maps.map # X and Y dims of the map

    mapDiff = int(input("Select map difficulty: \n1: Easy \n2: Medium \n3: Hard \n"))
    difficulties = [maps.easy, maps.med, maps.diff]
    walls = difficulties[mapDiff + 1] # Specific map walls

    # Grab the dimensions.
    rows = map[0]
    cols = map[1]

    # Set up the visual grid.
    visual = VisualGrid(rows, cols)

    # Draw walls
    for i in range(len(walls)):
        for j in range(len(walls[i])-1):
            visual.wall(walls[i][j], walls[i][j+1])

    # Grab/mark the start/goal.
    start = (0,0)
    treat = (7,7)
    catpng = plt.imread("cat_icon.png")
    treatpng = plt.imread("fish_icon.png")
    visual.image(start[0], start[1], catpng)
    visual.image(treat[0],  treat[1],  treatpng)
    visual.show(wait="Hit return to start")
    
    humanPath = [Node.Node(0,3,0), Node.Node(3,2,0.5), Node.Node(4,2,1), Node.Node(4,1,1.5), Node.Node(4,0,2)]
    tmax = 2
    dt = 0.5

    # Pre-allocate all nodes (for all x,y coord and all time t)
    nodes  = []
    for row in range(rows):
        for col in range(cols):
            for t in range(int(tmax/dt)):
                # Create node for each space
                node = Node.Node(row, col, t)
                nodes.append(node)
                # Add neighbors if there is still time left
                if tmax - t >= dt: 
                    node.neighbors = node.findNeighbors()


    cat = Node.Node(start[0], start[1], 0)
    human = humanPath[0]
    print (cat.lineOfSight(human))  

    
    

    

    