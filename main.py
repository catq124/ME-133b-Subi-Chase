from visualgrid import VisualGrid
import matplotlib.pyplot as plt
from math import inf
import maps 
import Node
import catplanner
import human_path


if __name__== "__main__":

    mapDim = maps.map # X and Y dims of the map

    # Get user input for map choice
    mapDiff = int(input("Select map difficulty: \n1: Easy \n2: Medium \n3: Hard \n"))
    difficulties = [maps.easy, maps.med, maps.diff]
    walls = difficulties[mapDiff - 1] # Specific map walls

    # Grab the dimensions.
    rows = mapDim[1]
    cols = mapDim[0]

    # Set up the visual grid.
    visual = VisualGrid(rows, cols)

    # Draw walls
    for i in range(len(walls)):
        for j in range(len(walls[i])-1):
            visual.wall(walls[i][j], walls[i][j+1])
    
    # Preset human path and corresponding tmax and dt
    # humanPath = [Node.Node(2,1,0), Node.Node(1,1,0.5), Node.Node(0,1,1), Node.Node(0,2,1.5), Node.Node(0,3,2)]
    # tmax = 2
    # dt = 0.5

    # Replace preset path with user-input for human goal node
    print(f"Choose human goal node between 0 ≤ row ≤ {rows-1} and 0 ≤ col ≤ {cols-1}: ")
    human_goal_row = int(input("Human goal row: "))
    human_goal_col = int(input("Human goal col: "))

    # Human start from map
    if mapDiff == 1:
        human_start_xy = maps.start_e
    elif mapDiff == 2:
        human_start_xy = maps.start_m
    else:
        human_start_xy = maps.start_d
    
    # Build human graph from human_path.py
    human_nodes = []

    for row in range(rows):
        for col in range(cols):
            human_nodes.append(human_path.Node(row, col))

    # Use cat neighbor logic to decide which moves are legal (i.e. walls)
    for node in human_nodes:
        temp = Node.Node(node.row, node.col, 0)
        legal = temp.findNeighbors(100)

        for neighbor in legal:
            if neighbor.row == node.row and neighbor.col == node.col:
                continue

            matches = [n for n in human_nodes
                       if n.row == neighbor.row and n.col == neighbor.col]
            if len(matches) > 0:
                node.neighbors.append(matches[0])

    # Grab/mark the start/goal
    hsrow, hscol = human_path.rows_cols(human_start_xy[0], human_start_xy[1], rows)
    hgrow, hgcol = human_goal_row, human_goal_col
    human_start = [n for n in human_nodes if (n.row, n.col) == (hsrow, hscol)][0]
    human_goal  = [n for n in human_nodes if (n.row, n.col) == (hgrow, hgcol)][0]

    # Plan the human path and return timed path for cat
    humanpath = human_path.planner(human_start, human_goal)

    if humanpath is None:
        raise RuntimeError("No human path found!")

    dt = 0.5
    humanPath = human_path.time(humanpath, dt)
    tmax = humanPath[-1].t

    # Pre-allocate all nodes (for all x,y coord and all time t)
    nodes  = []
    for row in range(rows):
        for col in range(cols):
            for i in range(int(tmax/dt)+1):
                t = i*dt
                nodes.append(Node.Node(row, col, t))
    
    # Add neighbors to all nodes
    for node in nodes:
        # Get neighbor list
        neighbors = node.findNeighbors(tmax)
        # Connect all neighbors
        for neighbor in neighbors:
            toAdd = next(n for n in nodes if n.row == neighbor.row and n.col == neighbor.col and n.t == neighbor.t)
            node.neighbors.append(toAdd)
                    
    # Define/mark the start/treat.
    startCoord = (0,0)
    treatCoord = (7,7)
    catpng = plt.imread("cat_icon.png")
    treatpng = plt.imread("fish_icon.png")
    humanpng = plt.imread("human_icon.png")
    visual.image(startCoord[0], startCoord[1], catpng)
    visual.image(treatCoord[0],  treatCoord[1],  treatpng)
    visual.image(humanPath[0].row, humanPath[0].col, humanpng)
    visual.show(wait="Hit return to start")

    # Set actual start and treat nodes
    start = next(n for n in nodes if n.row == startCoord[0] and n.col == startCoord[1] and n.t == 0)
    treat  = next(n for n in nodes if n.row == treatCoord[0] and n.col == treatCoord[1] and n.t == 0)
    treat.t = inf

    # Run catplanner algorithm and display steps
    catPath = catplanner.catPlanner1(start, treat, humanPath)
    for i in range(min(len(catPath), len(humanPath))):
        visual.image(catPath[i].row, catPath[i].col, catpng)
        visual.image(humanPath[i].row, humanPath[i].col, humanpng)
        print("CStep ", i, ": ", catPath[i].row, catPath[i].col)
        print("HStep ", i, ": ", humanPath[i].row, humanPath[i].col)
        visual.show(wait="Hit return for next step")
    

    