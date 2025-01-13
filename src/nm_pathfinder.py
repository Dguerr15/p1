from heapq import heappop, heappush
from math import inf, sqrt

def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """
    path = []
    boxes = {}

    source_box = None
    destination_box = None
    for box in mesh['boxes']:
        x1, x2, y1, y2 = box
        if x1 <= source_point[0] <= x2 and y1 <= source_point[1] <= y2:
            source_box = box
            boxes[box] = 0
        if x1 <= destination_point[0] <= x2 and y1 <= destination_point[1] <= y2:
            destination_box = box
            boxes[box] = 0
        if (len(boxes) == 2):
            break
    
    if source_box == destination_box:
        return [source_point, destination_point], boxes.keys()

    #queue = []
    frontier = []
    frontier.append(source_box)
    came_from = {} 
    came_from[source_box] = None
    
    finished = False
    while not(len(frontier) == 0):
        current = frontier.pop()
        for next in mesh['adj'][current]:
            if next not in came_from:
                frontier.append(next)
                came_from[next] = current
                if next == destination_box:
                    finished = True
                    break
    
    if not finished:
        print("No path found")
        return [], boxes.keys()
    
    path.append(destination_point)
    current = came_from[destination_box]
    while current != source_box:
        x1, x2, y1, y2 = current
        next = came_from[current]
        x3, x4, y3, y4 = next
        new_x = (max(x1, x3), min(x2, x4))
        new_y = (max(y1, y3), min(y2, y4))
            

        path.append((new_x,new_y))
        current = next
        
    path.append(source_point)
    path.reverse()
        
        # # investigate children
        # for box, adj in mesh:
        #     # calculate cost along this path to child
        #     cost_to_child = priority + transition_cost(graph, cell, child)  # heuristic(source_point, destination_point)
        #     if child not in pathcosts or cost_to_child < pathcosts[child]:
        #         pathcosts[child] = cost_to_child            # update the cost
        #         paths[child] = cell                         # set the backpointer
        #         heappush(queue, (cost_to_child, child))     # put the child on the priority queue
            
    
    return path, came_from.keys()
    #return path, boxes.keys()

def heuristic(source_point, destination_point):
    """
    Calculates the heuristic cost between two points
    """
    return abs(source_point[0] - destination_point[0]) + abs(source_point[1] - destination_point[1])
