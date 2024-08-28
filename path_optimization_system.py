# Graph Optimization

#
# Finding shortest paths to drive from home to work on a road network
#

from graph import DirectedRoad, Node, RoadMap

def create_graph(map_filename):
    """
    Parses the map file and constructs a road map (graph).

    Travel time and traffic multiplier should be each cast to a float.

    Parameters:
        map_filename : str
            Name of the map file.

    Assumes:
        Each entry in the map file consists of the following format, separated by spaces:
            source_node destination_node travel_time road_type traffic_multiplier

        Note: hill road types always are uphill in the source to destination direction and
              downhill in the destination to the source direction. Downhill travel takes
              1/4 as long as uphill travel. The travel_time represents the time to travel
              from source to destination (uphill).

        e.g.
            N0 N1 10 highway 1
        This entry would become two directed roads; one from 'N0' to 'N1' on a highway with
        a weight of 10.0, and another road from 'N1' to 'N0' on a highway using the same weight.

        e.g.
            N2 N3 7 uphill 2
        This entry would become two directed roads; one from 'N2' to 'N3' on a hill road with
        a weight of 7.0, and another road from 'N3' to 'N2' on a hill road with a weight of 1.75.
        Note that the directed roads created should have both type 'hill', not 'uphill'!

    Returns:
        RoadMap
            A directed road map representing the given map.
    """
    map_file = open(map_filename)
    map_file = map_file.readlines()

    #creates a roads list of DirectRoad objects
    roads = []
    for road in map_file:
        road = road.split()

        source = Node(road[0])
        dest = Node(road[1])
        travel_time = road[2]
        road_type = road[3]
        traffic = road[4]

        if road_type == "uphill":
            roads.append(DirectedRoad(source,dest,float(travel_time),"hill",float(traffic)))
            roads.append(DirectedRoad(dest,source,float(travel_time)/4,"hill",float(traffic)))
                #switches the source and dest node around and time from dest to start is time/4
        else:
            roads.append(DirectedRoad(source,dest,float(travel_time),road_type,float(traffic)))
            roads.append(DirectedRoad(dest,source,float(travel_time),road_type,float(traffic)))

    map = RoadMap()
    for road in roads:
        source = road.get_source_node()
        dest = road.get_destination_node()

        all_nodes = map.get_all_nodes()
        if source not in all_nodes:
            map.insert_node(source)
        if dest not in all_nodes:
            map.insert_node(dest)

        #put every road into the map
        map.insert_road(road)

    return map


def find_shortest_path(roadmap, start, end, restricted_roads=None, has_traffic=False):
    """
    Finds the shortest path between start and end nodes on the road map,
    without using any restricted roads, following traffic conditions.
    If restricted_roads is None, assume there are no restricted roads.
    Use Dijkstra's algorithm.

    Parameters:
        roadmap: RoadMap
            The graph on which to carry out the search.
        start: Node
            Node at which to start.
        end: Node
            Node at which to end.
        restricted_roads: list of str or None
            Road Types not allowed on path. If None, all are roads allowed
        has_traffic: bool
            Flag to indicate whether to get shortest path during traffic or not.

    Returns:
        A two element tuple of the form (best_path, best_time).
            The first item is a list of Node, the shortest path from start to end.
            The second item is a float, the length (time traveled) of the best path.
        If there exists no path that satisfies constraints, then return None.
    """
    #special cases
    all_nodes = roadmap.get_all_nodes()
    if start not in all_nodes and end not in all_nodes:
        return None

    if start == end:
        return ([start],0)

    unvisited = all_nodes

    #shortest dictionary to keep track of travel times across nodes
    shortest = {}
    for node in all_nodes:
        shortest[node] = float('inf')
    shortest[start] = 0

    previous = {node: None for node in all_nodes}
    while unvisited:
        current_node = min(unvisited, key = lambda node: shortest[node]) #current node initially is the unvisited node with the lowest travel time from the start

        if shortest[current_node] == float('inf'):
            break

        if current_node == end:
            break

        #run through the roads to neighboring nodes and add up total times
        reachable_roads = roadmap.get_reachable_roads_from_node(current_node, restricted_roads)
        for road in reachable_roads:
            neighbor = road.get_destination_node()
            time = shortest[current_node] + road.get_travel_time(has_traffic)

            #if the calculated time is the current shortest time, update the shortest dict and the previous dict
            if time < shortest[neighbor]:
                shortest[neighbor] = time
                previous[neighbor] = current_node

        #current node is now visited, so can remove from unvisited
        unvisited.remove(current_node)

    shortest_path = []
    current_node = end
    #keep adding the nodes from the end to the start which finds the node path with shortest time
    try:
        while previous[current_node]:
            shortest_path.insert(0,current_node)
            current_node = previous[current_node]

        if shortest_path:
            shortest_path.insert(0,current_node)
        if shortest_path == []: #if no path exists from start to end, return None
            return None
        return (shortest_path, shortest[end])
    except: #when impossible path (no highway)
        return None


def find_shortest_path_no_traffic(filename, start, end):
    """
    Finds the shortest path from start to end during conditions of no traffic.

    You must use find_shortest_path.

    Parameters:
        filename: str
            Name of the map file that contains the graph
        start: Node
            Node object at which to start.
        end: Node
            Node object at which to end.

    Returns:
        list of Node
            The shortest path from start to end in normal traffic.
        If there exists no path, then return None.
    """
    map = create_graph(filename)
    shortest_path = (find_shortest_path(map, start, end, [], False))
    return shortest_path[0]


def find_shortest_path_restricted(filename, start, end):
    """
    Finds the shortest path from start to end when local roads and hill roads cannot be used.

    You must use find_shortest_path.

    Parameters:
        filename: str
            Name of the map file that contains the graph
        start: Node
            Node object at which to start.
        end: Node
            Node object at which to end.

    Returns:
        list of Node
            The shortest path from start to end given the aforementioned conditions.
        If there exists no path that satisfies constraints, then return None.
    """
    map = create_graph(filename)
    shortest_path = (find_shortest_path(map, start, end, ["local", "hill"], False))
    return shortest_path[0]


def find_shortest_path_in_traffic_no_toll(filename, start, end):
    """
    Finds the shortest path from start to end when toll roads cannot be used and in traffic,
    i.e. when all roads' travel times are multiplied by their traffic multipliers.

    You must use find_shortest_path.

    Parameters:
        filename: str
            Name of the map file that contains the graph
        start: Node
            Node object at which to start.
        end: Node
            Node object at which to end.

    Returns:
        list of Node
            The shortest path from start to end given the aforementioned conditions.
        If there exists no path that satisfies the constraints, then return None.
    """
    map = create_graph(filename)
    shortest_path = (find_shortest_path(map, start, end, ["toll"], True))
    return shortest_path[0]


if __name__ == '__main__':

    # UNCOMMENT THE LINES BELOW TO DEBUG OR TO EXECUTE
    pass

    small_map = create_graph('./maps/small_map.txt')

    # ------------------------------------------------------------------------
    road_map = create_graph("maps/test_create_graph.txt")
    print(road_map)
    # ------------------------------------------------------------------------

    start = Node('N0')
    end = Node('N4')
    restricted_roads = []
    print(find_shortest_path(road_map, start, end, restricted_roads))
