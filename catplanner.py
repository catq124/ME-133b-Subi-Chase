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

# Assume some function nodeAtTime(self, path, time) return Node
# in human path.py

def catPlanner1(start, goal, hpath, nodes):
    t = 0
    tmax = hpath[len(hpath)-1].t
    start.seen   = True
    start.cost   = 0
    start.parent = None
    onDeck = [start]
    path = []
    
    while t<=tmax:
        t =+ 0.5
        
        if not (len(onDeck) > 0):
            return None
            
        node = onDeck.pop(0)
        
        node.done = True
        if goal.done:
            break
        
        for neighbor in node.neighbors:
            if neighbor.done:
                continue
            if neighbor.row == node.row and neighbor.col == node.col:
                cost = node.cost
            else: cost = node.cost + 1

            if neighbor.seen:
                if neighbor.cost <= cost:
                    continue
                else:
                    onDeck.remove(neighbor)
            
            neighbor.seen = True
            neighbor.cost = cost
            neighbor.parent = node
            bisect.insort(onDeck, neighbor)

    path = [goal]
    while path[0].parent:
        path.insert(0, path[0].parent)
    return path

def catPlanner2():
    
    return