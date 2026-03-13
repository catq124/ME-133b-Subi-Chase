"""
catplanner.py 

Containes functions for different path planning algorithms adjusted for temporal
planning. At end should be able to report each algorithms cost, how many nodes had to be checked
and the time need to reach each path. Functions will either return the path to the treat
or return None meaning that the cat could not find an optimal path in the given time

Functions:
    - catPlanner1(): dijkstras
    - catPlanner2(): A*
"""

import bisect
from math import inf

# Assume some function nodeAtTime(self, path, time) return Node
# in human path.py

# Returns list of Nodes from start --> end (end going towards goal, as time allots) 
def catPlanner1(start, goal, hpath, safeDist = 2):
    # Define variables
    dt = hpath[1].t
    tmax = hpath[len(hpath)-1].t
    start.seen   = True
    start.cost   = 0
    start.parent = None
    onDeck = [start]
    path = []
    finalNodes = []
    
    while True: # All print statements are just for debugging
        print("*************************************")
        # Break out of loop if onDeck empty
        if not (len(onDeck) > 0):
            print("onDeck empty")
            break
            
        node = onDeck.pop(0)
        print("Node: ", node.row, node.col, node.t)

        node.done = True
        # Break out of loop if node is at goal
        if node.row == goal.row and node.col == goal.col:
            # Set final nodes to only node/goal
            finalNodes = [node]
            break

        # Add node to list of final nodes if at tmax (ie. path could end here)
        if node.t == tmax:
            finalNodes.append(node)

        print("Neighbor list: ", node.neighbors)
        for neighbor in node.neighbors:
            print("Neighbor: ", neighbor.row, neighbor.col, neighbor.t)
            # Skip if already done or would be out of time
            if neighbor.done or neighbor.t > tmax:
                print("Neighbor done")
                continue
            # Get human node at this neighbor's time
            hnode = hpath[int(neighbor.t/dt)]
            print("Hnode: ", hnode.row, hnode.col)
            # Skip if neighbor would be caught
            if neighbor.distance(hnode) < safeDist and neighbor.lineOfSight(hnode):
                print("Line of sight: ", neighbor.lineOfSight(hnode))
                print("Neighbor would get caught")
                continue
            # Define neighbor cost
            if neighbor.row == node.row and neighbor.col == node.col:
                cost = node.cost
            else: cost = node.cost + 1
            print("Neighbor cost: ", cost)
            
            # Deal with seen neighbors
            if neighbor.seen:
                # Skip if existing cost lower
                if neighbor.cost <= cost:
                    print("Lower neighbor cost already exists")
                    continue
                # Replace (ie. remove existing) if new cost lower
                else:
                    print("Removed existing neighbor")
                    onDeck.remove(neighbor)
            
            neighbor.seen = True
            neighbor.cost = cost
            neighbor.parent = node
            print("Neighbor added")
            bisect.insort(onDeck, neighbor)
            print("onDeck: ", onDeck)
            print("---------------------------------------")

    # Determine which final node to use (ie. closest to final goal)
    print("Final Nodes: ", finalNodes) 
    mindist = inf
    finalNode = None
    for node in finalNodes:
        if node.distance(goal) < mindist:
            mindist = node.distance(goal)
            finalNode = node
            
    # Create and return path
    path = [finalNode]
    while path[0].parent:
        path.insert(0, path[0].parent)
    return path

def catPlanner2():
    
    return