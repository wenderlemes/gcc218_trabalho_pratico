"""Modified code from https://developers.google.com/optimization/routing/tsp#or-tools """
# Copyright Matthew Mack (c) 2020 under CC-BY 4.0: https://creativecommons.org/licenses/by/4.0/

from __future__ import print_function
import math
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from PIL import Image, ImageDraw
import os
import time
import copy
from itertools import permutations

# Change these file names to the relevant files.
ORIGINAL_IMAGE = "images/ww-5000-stipple.png"
IMAGE_TSP = "images/ww-5000-stipple.tsp"
NUMBER_OF_POINTS = 4989
NUMBER_OF_PARTITIONS = 8
INITIAL_VERTEX = 0

def create_data_model():
    """Stores the data for the problem."""
    # Extracts coordinates from IMAGE_TSP and puts them into an array
    list_of_nodes = []
    with open(IMAGE_TSP) as f:
        for _ in range(6):
            next(f)
        for line in f:
            i,x,y = line.split()
            list_of_nodes.append((int(float(x)),int(float(y))))
    data = {}
    # Locations in block units
    data['locations'] = list_of_nodes # yapf: disable
    data['num_vehicles'] = 1
    data['depot'] = 0
    return data

def compute_euclidean_distance_matrix(locations):
    """Creates callback to return distance between points."""
    distances = {}
    for from_counter, from_node in enumerate(locations):
        distances[from_counter] = {}
        for to_counter, to_node in enumerate(locations):
            if from_counter == to_counter:
                distances[from_counter][to_counter] = 0
            else:
                # Euclidean distance
                distances[from_counter][to_counter] = (int(
                    math.hypot((from_node[0] - to_node[0]),
                               (from_node[1] - to_node[1]))))
    return distances

def print_solution(manager, routing, solution):
    """Prints solution on console."""
    print('Objective: {}'.format(solution.ObjectiveValue()))
    index = routing.Start(0)
    plan_output = 'Route:\n'
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += ' {} ->'.format(manager.IndexToNode(index))
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += ' {}\n'.format(manager.IndexToNode(index))
    print(plan_output)
    plan_output += 'Objective: {}m\n'.format(route_distance)

def get_routes(solution, routing, manager):
    """Get vehicle routes from a solution and store them in an array."""
    # Get vehicle routes and store them in a two dimensional array whose
    # i,j entry is the jth location visited by vehicle i along its route.
    routes = []
    for route_nbr in range(routing.vehicles()):
      index = routing.Start(route_nbr)
      route = [manager.IndexToNode(index)]
      #while not routing.IsEnd(index):
      #  index = solution.Value(routing.NextVar(index))
      counter = 0
      while counter < len(solution):
        counter += 1
        index = solution[index]
        route.append(manager.IndexToNode(index))
      routes.append(route)
    return routes[0]

def draw_routes(nodes, path):
    """Takes a set of nodes and a path, and outputs an image of the drawn TSP path"""
    tsp_path = []
    for location in path:
        tsp_path.append(nodes[int(location)])

    original_image = Image.open(ORIGINAL_IMAGE)
    width, height = original_image.size

    tsp_image = Image.new("RGBA",(width,height),color='white')
    tsp_image_draw = ImageDraw.Draw(tsp_image)
    #tsp_image_draw.point(tsp_path,fill='black')
    tsp_image_draw.line(tsp_path,fill='black',width=1)
    tsp_image = tsp_image.transpose(Image.FLIP_TOP_BOTTOM)
    FINAL_IMAGE = IMAGE_TSP.replace("-stipple.tsp","-tsp.png")
    tsp_image.save(FINAL_IMAGE)
    print("TSP solution has been drawn and can be viewed at", FINAL_IMAGE)

def nearest_neighbors_solution(distance_matrix):
    visited = {i: False for i in range(NUMBER_OF_POINTS)}
    nearest_neighbors = {i: -1 for i in range(NUMBER_OF_POINTS)}
    last_vertex = INITIAL_VERTEX
    should_continue = True

    while should_continue:
        should_continue = False
        visited[last_vertex] = True
        shortest_distance = float("inf")
        closest_neighbor = -1

        for i in distance_matrix[last_vertex]:
            if distance_matrix[last_vertex][i] < shortest_distance and not (visited[i]):
                shortest_distance = distance_matrix[last_vertex][i]
                closest_neighbor = i
                should_continue = True

        if should_continue:
            nearest_neighbors[last_vertex] = closest_neighbor
            last_vertex = closest_neighbor
        else:
            nearest_neighbors[last_vertex] = INITIAL_VERTEX
    return nearest_neighbors

def two_opt_solution(distance_matrix):
    solution = nearest_neighbors_solution(distance_matrix)
    original_group = convert_solution_to_group(solution)
    partitions = NUMBER_OF_PARTITIONS

    while(partitions > 0):
        two_opt(distance_matrix, original_group, partitions)
        partitions = int(partitions / 2)

    new_solution = convert_group_to_solution(original_group)

    return new_solution

def two_opt(distance_matrix, group, partitions):
    partition_size = int(len(group)/partitions)

    for k in range(partitions):
        while True:
            min_change = 0
            min_i = -1
            min_j = -1
            for i in range(1 + (k*partition_size), ((k+1)*partition_size)-2):
                for j in range(i+1, ((k+1)*partition_size)):
                    u = group[i-1]
                    v = group[i]
                    w = group[j]
                    x = group[(j+1) % ((k+1)*partition_size)]
                    current_distance = (distance_matrix[u][v] + distance_matrix[w][x])
                    new_distance = (distance_matrix[u][w] + distance_matrix[v][x])
                    change = new_distance - current_distance

                    if change < min_change:
                        min_change = change
                        min_i = i
                        min_j = j

            swap_edges(group, min_i, min_j)
            if min_change == 0:
                break
            print(min_change)

def swap_edges(group, v, w):
    #Reverses the entire slice, from vertex v to vertex w (including v and w)
    group[v:w+1] = group[v:w+1][::-1]

def convert_group_to_solution(group):
    solution = {}

    for i in range(len(group)-1):
        solution[group[i]] = group[i+1]

    solution[group[-1]] = NUMBER_OF_POINTS
    print(solution)
    return solution

def convert_solution_to_group(solution):
    head = INITIAL_VERTEX
    group = []

    for i in range(NUMBER_OF_POINTS):
        group.append(head)
        head = solution[head]

    return group

def calculate_group_cost(distance_matrix, group):
    cost = 0
    for i in range(len(group)):
        cost += distance_matrix[group[i]][group[(i+1) % len(group)]]
    return cost

def main():
    """Entry point of the program."""
    starting_moment = time.time()

    # Instantiate the data problem.
    print("Step 1/5: Initialising variables")
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['locations']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)
    print("Step 2/5: Computing distance matrix")
    distance_matrix = compute_euclidean_distance_matrix(data['locations'])

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    print("Step 3/5: Setting an initial solution")
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    print("Step 4/5: Solving")
    #solution = routing.SolveWithParameters(search_parameters)
    #solution = nearest_neighbors_solution(distance_matrix)
    solution = two_opt_solution(distance_matrix)

    # Print solution on console.
    if solution:
        #print_solution(manager, routing, solution)
        print("Step 5/5: Drawing the solution")
        routes = get_routes(solution, routing, manager)
        draw_routes(data['locations'], routes)
    else:
        print("A solution couldn't be found :(")

    finishing_moment = time.time()
    print("Total time elapsed during execution: " + str(finishing_moment - starting_moment) + " seconds")
    print("Total distance: " + str(calculate_group_cost(distance_matrix, convert_solution_to_group(solution))))
if __name__ == '__main__':
    main()