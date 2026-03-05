from visualgrid import VisualGrid
import matplotlib.pyplot as plt


map = (10, 10)
walls = [[(1,1), (4,1), (4,2)],
         [(3,2), (3,5), (6,5)]]    # Each list is a continuous line

WHITE  = [1.000, 1.000, 1.000]
BLACK  = [0.000, 0.000, 0.000]
RED    = [1.000, 0.000, 0.000]
BARK   = [0.325, 0.192, 0.094]  # bark brown
GREEN  = [0.133, 0.545, 0.133]  # forrest green
SKY    = [0.816, 0.925, 0.992]  # light blue

if __name__== "__main__":
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
    #start = [n for n in nodes if map[n.row][n.col] in 'Ss'][0]
    #goal  = [n for n in nodes if map[n.row][n.col] in 'Gg'][0]
    start = (0,0)
    treat = (9,9)
    catpng = plt.imread("cat_icon.png")
    treatpng = plt.imread("fish_icon.png")
    visual.image(start[0], start[1], catpng)
    visual.image(treat[0],  treat[1],  treatpng)
    visual.show(wait="Hit return to start")