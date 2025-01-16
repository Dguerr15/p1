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
    explored_boxes = set()

    source_box = None
    destination_box = None

    for box in mesh['boxes']:
        x1, x2, y1, y2 = box
        if x1 <= source_point[0] <= x2 and y1 <= source_point[1] <= y2:
            source_box = box
            explored_boxes.add(box)
        if x1 <= destination_point[0] <= x2 and y1 <= destination_point[1] <= y2:
            destination_box = box
            explored_boxes.add(box)
        if source_box and destination_box:
            break

    if source_box == destination_box:
        return [source_point, destination_point], explored_boxes

    # Priority queue for bidirectional A*
    frontier = []
    heappush(frontier, (0, source_box, 'destination'))  # (priority, box, goal)
    heappush(frontier, (0, destination_box, 'source'))

    # Cost and path dictionaries for both directions
    forward_cost = {source_box: 0}
    backward_cost = {destination_box: 0}
    forward_prev = {source_box: None}
    backward_prev = {destination_box: None}
    forward_detail_points = {source_box: source_point}
    backward_detail_points = {destination_box: destination_point}

    meeting_point = None

    while frontier:
        priority, current, goal = heappop(frontier)

        if goal == 'destination':
            current_cost = forward_cost
            current_prev = forward_prev
            current_detail_points = forward_detail_points
            opposite_prev = backward_prev
            end_point = destination_point
        else:
            current_cost = backward_cost
            current_prev = backward_prev
            current_detail_points = backward_detail_points
            opposite_prev = forward_prev
            end_point = source_point

        if current in opposite_prev:
            meeting_point = current
            # Add a detail point constrained to the meeting box before breaking
            current_point = current_detail_points[current]
            constrained_point = constrain_to_box(current_point, current)
            forward_detail_points[current] = constrained_point
            break

        for next_box in mesh['adj'].get(current, []):
            current_point = current_detail_points[current]
            constrained_point = constrain_to_box(current_point, next_box)
            new_cost = current_cost[current] + box_distance(current, next_box)

            if next_box not in current_cost or new_cost < current_cost[next_box]:
                current_cost[next_box] = new_cost
                heuristic_cost = heuristic(box_center(next_box), end_point)
                heappush(frontier, (new_cost + heuristic_cost, next_box, goal))
                current_prev[next_box] = current
                current_detail_points[next_box] = constrained_point

    if not meeting_point:
        print("No path found")
        return [], explored_boxes

    # Reconstruct the path
    path = reconstruct_path(forward_prev, backward_prev, forward_detail_points, backward_detail_points, meeting_point)

    boxes = list(forward_prev.keys()) + list(backward_prev.keys())
    return path, boxes

def heuristic(source_point, destination_point):
    """
    Calculates the heuristic cost between two points
    """
    return abs(source_point[0] - destination_point[0]) + abs(source_point[1] - destination_point[1])

def box_distance(box1, box2):
    """
    Calculates the approximate distance between two boxes (centroid distance).
    """
    x1, x2, y1, y2 = box1
    x3, x4, y3, y4 = box2
    center1 = ((x1 + x2) / 2, (y1 + y2) / 2)
    center2 = ((x3 + x4) / 2, (y3 + y4) / 2)
    return sqrt((center1[0] - center2[0]) ** 2 + (center1[1] - center2[1]) ** 2)

def box_center(box):
    """
    Returns the center point of a box.
    """
    x1, x2, y1, y2 = box
    return ((x1 + x2) / 2, (y1 + y2) / 2)

def reconstruct_path(forward_prev, backward_prev, forward_detail_points, backward_detail_points, meeting_point):
    """
    Reconstructs the path from the forward and backward pointers.
    """
    path = []

    # Path from source to meeting point
    current = meeting_point
    while current is not None:
        if forward_prev[current] is None:  # This is the source box
            # Add an intermediate detail point constrained within the source box
            if path:
                first_point = path[-1]
            else:
                first_point = forward_detail_points[current]
            constrained_point = constrain_to_box(first_point, current)
            path.append(constrained_point)

        path.append(forward_detail_points[current])
        current = forward_prev[current]

    path.reverse()

    # Path from meeting point to destination
    current = backward_prev[meeting_point]
    while current is not None:  
        # Add an intermediate detail point constrained within the destination box
        if current in backward_detail_points and current == list(backward_prev.keys())[0]:  # Destination box
            last_point = path[-1]
            intermediate_point = constrain_to_box(last_point, current)
            path.append(intermediate_point)

        path.append(backward_detail_points[current])
        current = backward_prev[current]

    return path

def constrain_to_box(point, box):
    """
    Constrains a point to lie within the bounds of a box.
    """
    x1, x2, y1, y2 = box
    x, y = point
    constrained_x = max(x1, min(x, x2))
    constrained_y = max(y1, min(y, y2))
    return (constrained_x, constrained_y)

# A* Search
 # path = []
    # explored_boxes = set()

    # source_box = None
    # destination_box = None
    # for box in mesh['boxes']:
    #     x1, x2, y1, y2 = box
    #     if x1 <= source_point[0] <= x2 and y1 <= source_point[1] <= y2:
    #         source_box = box
    #         explored_boxes.add(box)
    #     if x1 <= destination_point[0] <= x2 and y1 <= destination_point[1] <= y2:
    #         destination_box = box
    #         explored_boxes.add(box)
    #     if (source_box and destination_box):
    #         break
    
    # if source_box == destination_box:
    #     return [source_point, destination_point], explored_boxes
    
    # frontier = []  # Min-heap for A*
    # heappush(frontier, (0, source_box))  # (priority, box)
    # came_from = {}
    # cost_so_far = {}

    # came_from[source_box] = None
    # cost_so_far[source_box] = 0

    # while frontier:
    #     _, current = heappop(frontier)

    #     if current == destination_box:
    #         break

    #     for next_box in mesh['adj'].get(current, []):
    #         # Calculate the cost to move to the next box
    #         new_cost = cost_so_far[current] + box_distance(current, next_box)

    #         if next_box not in cost_so_far or new_cost < cost_so_far[next_box]:
    #             cost_so_far[next_box] = new_cost
    #             priority = new_cost + heuristic(box_center(next_box), destination_point)
    #             heappush(frontier, (priority, next_box))
    #             came_from[next_box] = current
    
    # try:
    #     came_from[destination_box]
    # except:
    #     print("No path found")
    #     return [], explored_boxes
    
    # # Reconstruct the path
    # path.append(destination_point)
    # current = came_from[destination_box]
    # while current != source_box:
    #     x1, x2, y1, y2 = current
    #     next_box = came_from[current]
    #     x3, x4, y3, y4 = next_box
    #     new_x = (max(x1, x3) + min(x2, x4)) / 2
    #     new_y = (max(y1, y3) + min(y2, y4)) / 2
    #     path.append((new_x, new_y))
    #     current = next_box

    # path.append(source_point)
    # path.reverse()
    # return path, came_from.keys()



# Breadth First Search
    # path = []
    # boxes = {}

    # source_box = None
    # destination_box = None
    # for box in mesh['boxes']:
    #     x1, x2, y1, y2 = box
    #     if x1 <= source_point[0] <= x2 and y1 <= source_point[1] <= y2:
    #         source_box = box
    #         boxes[box] = 0
    #     if x1 <= destination_point[0] <= x2 and y1 <= destination_point[1] <= y2:
    #         destination_box = box
    #         boxes[box] = 0
    #     if (len(boxes) == 2):
    #         break
    
    # if source_box == destination_box:
    #     return [source_point, destination_point], boxes.keys()

    # #queue = []
    # frontier = [] # functioning as a queue
    # frontier.append(source_box)
    # came_from = {} 
    # came_from[source_box] = None
    
    # finished = False
    # while not(len(frontier) == 0):
    #     current = frontier.pop(0)
    #     for next in mesh['adj'][current]:
    #         if next not in came_from:
    #             frontier.append(next)
    #             came_from[next] = current
    #             if next == destination_box:
    #                 finished = True
    #                 break
    
    # if not finished:
    #     print("No path found")
    #     return [], boxes.keys()
    
    # path.append(destination_point)
    # current = came_from[destination_box]
    # while current != source_box:
    #     x1, x2, y1, y2 = current
    #     next = came_from[current]
    #     x3, x4, y3, y4 = next
    #     new_xRange = (max(x1, x3), (max(x1,x3) + min(x2,x4))/2,min(x2, x4))
    #     new_yRange = (max(y1, y3), (max(y1,y3) + min(y2,y4))/2,min(y2, y4))

    #     distance = inf

    #     new_x = 0
    #     new_y = 0
    #     for i in new_xRange:
    #         for j in new_yRange:
    #             dist = sqrt((path[-1][0] - i)**2 + (path[-1][1] - j)**2)
    #             if dist < distance:
    #                 distance = dist
    #                 new_x = i
    #                 new_y = j
            

    #     path.append((new_x,new_y))
    #     current = next
        
    # path.append(source_point)
    # path.reverse()    
    # return path, came_from.keys()