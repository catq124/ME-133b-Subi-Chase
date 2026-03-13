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
    
    humanPath = [Node.Node(2,1,0), Node.Node(1,1,0.5), Node.Node(0,1,1), Node.Node(0,2,1.5), Node.Node(0,3,2)]
    tmax = 2
    dt = 0.5

    # Pre-allocate all nodes (for all x,y coord and all time t)
    nodes  = []
    for row in range(rows):
        for col in range(cols):
            for i in range(int(tmax/dt)+1):
                t = i*dt
                nodes.append(Node.Node(row, col, t))
    
    for node in nodes:
        # Get neighbor list
        neighbors = node.findNeighbors(tmax)
        # Connect all neighbors
        for neighbor in neighbors:
            toAdd = next(n for n in nodes if n.row == neighbor.row and n.col == neighbor.col and n.t == neighbor.t)
            node.neighbors.append(toAdd)
                    
    # Grab/mark the start/goal.
    startCoord = (0,0)
    treatCoord = (7,7)
    catpng = plt.imread("cat_icon.png")
    treatpng = plt.imread("fish_icon.png")
    humanpng = plt.imread("human_icon.png")
    visual.image(startCoord[0], startCoord[1], catpng)
    visual.image(treatCoord[0],  treatCoord[1],  treatpng)
    visual.image(humanPath[0].row, humanPath[0].col, humanpng)
    visual.show(wait="Hit return to start")

    start = next(n for n in nodes if n.row == startCoord[0] and n.col == startCoord[1] and n.t == 0)
    treat  = next(n for n in nodes if n.row == treatCoord[0] and n.col == treatCoord[1] and n.t == 0)
    treat.t = inf

    catPath = catplanner.catPlanner1(start, treat, humanPath)
    for i in range(len(catPath)):
        visual.image(catPath[i].row, catPath[i].col, catpng)
        visual.image(humanPath[i].row, humanPath[i].col, humanpng)
        print("CStep ", i, ": ", catPath[i].row, catPath[i].col)
        print("HStep ", i, ": ", humanPath[i].row, humanPath[i].col)
        visual.show(wait="Hit return for next step")

    

    