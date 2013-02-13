#!/usr/bin/python -u
import sys
import math 
#from w_utils import *

max_cost = 1000 #1000000000000
debug = False #True #True
express_flag = False#True #False#False
cost_building = 1.0  #1.0
cost_wall = 1.0  #1.0
cost_normal = 1/30 #.8  #0.3
cost_road = -0.8/30#0.8/30.8  #0.1

def get_square_from_float( coords ):
    x = int( round( coords[0], 0 ) )
    y = int( round( coords[1], 0 ) )
    return [ x, y ]


def find_intersecting_squares( start, end ):
    startf = [ float(x) for x in start ]
    endf = [ float(x) for x in end ]
    y1 = startf[1]
    y2 = endf[1]
    x1 = startf[0]
    x2 = endf[0]
    
    #current_square = get_square_from_float( startf )
    end_square = get_square_from_float( endf )
    endx = end_square[0]
    endy = end_square[1]

    start_square = get_square_from_float( startf )
    startx = start_square[0]
    starty = start_square[1]

    intersecting_squares = []
    if startx == endx:
        y = starty + 1
        while y < endy:
            intersecting_squares.append( [ startx, y ] )
            y += 1
    elif startx < endx:
        x = startx + 1
        while x < endx:
            y = int( float( ( endy - starty ) ) / ( endx - startx ) * ( float(x) - startx ) + starty )
            intersecting_squares.append( [ x, y ] )
            x += 1
    else:
        x = endx + 1
        while x < startx:
            y = int( float( ( endy - starty ) ) / ( endx - startx ) * ( float(x) - startx ) + starty )
            intersecting_squares.append( [ x, y ] )
            x += 1

    return intersecting_squares

def check_express( current_pos, end, map_list ):
    # check if a straight line can go from there to the end
    candidates_to_check = find_intersecting_squares( current_pos, end )
    for [ x, y ] in candidates_to_check:
        if map_list[x][y] == "B" or map_list[x][y] == "#":
            return False
    
    return True


def write_output( string_output ):
    sys.stdout.write( string_output + "\n" )

def delta(a,b,c):
    d = pow((c[0]-b[0]) - (b[0] - a[0]),2) + pow((c[1] - b[1]) - (b[1]-a[1]),2)
    return d

def get_start_end( t ):
    t = t.split()
    start_x = int( t[0] )
    end_x = int( t[2] )
    start_y = int( t[1] )
    end_y = int( t[3] )
    return [ start_x, start_y ], [ end_x, end_y ]

def modify_start_end( int_start, int_end ):
    add_val = 0.5 - 0.1 / 10**5
    #add_val = 0.0

    start = [ float(i) for i in int_start ]
    end = [ float(i) for i in int_end ]

    if int_start[0] < int_end[0]:
        start[0] = float( int_start[0] ) + add_val
        end[0] = float( int_end[0] ) - add_val
    elif int_start[0] > int_end[0]:
        start[0] = float( int_start[0] ) - add_val
        end[0] = float( int_end[0] ) + add_val

    if int_start[1] < int_end[1]:
        start[1] = float( int_start[1] ) + add_val
        end[1] = float( int_end[1] ) - add_val
    elif int_start[1] > int_end[1]:
        start[1] = float( int_start[1] ) - add_val
        end[1] = float( int_end[1] ) + add_val

    return [ start, end ]

def greedy( current_pos, int_end, map_list ):
    next_pos_vector = int_end[:]
    next_pos = int_end[:]
    greedy_direction = [ int_end[0] - current_pos[0], int_end[1] - current_pos[1] ]
    
    if greedy_direction[0] > 0:
        next_pos_vector[0] = 1
        next_pos[0] = current_pos[0] + 1
    elif greedy_direction[0] < 0:
        next_pos_vector[0] = -1
        next_pos[0] = current_pos[0] - 1
    else: 
        next_pos_vector[0] = 0
        next_pos[0] = current_pos[0]

    if greedy_direction[1] > 0:
        next_pos_vector[1] = 1
        next_pos[1] = current_pos[1] + 1
    elif greedy_direction[1] < 0:
        next_pos_vector[1] = -1
        next_pos[1] = current_pos[1] - 1
    else:
        next_pos_vector[1] = 0
        next_pos[1] = current_pos[1]

    # check if there is a building there
    if next_pos_vector == [ 0,0 ]:
        next_pos = None
    elif next_pos_vector[0] == 0:
        # must move vertically; check if there is a building there
        if map_list[ next_pos[0] ][ next_pos[1] ] == 'B':
            # if there is a building there, just fly over it to destination
            next_pos = None
        else:
            # proceed vertically
            pass
    elif next_pos_vector[1] == 0:
        # must move horizontally
        if map_list[ next_pos[0] ][ next_pos[1] ] == 'B':
            # if there is a building there, just fly over it to destination
            next_pos = None
        else:
            # proceed horizontally
            pass

    if next_pos == current_pos:
        next_pos = None

    return next_pos

def print_pos_in_map(l_map, current_node, goal_node):
    map_str = ""
    width, height = get_map_size(l_map)
    
    '''current_node.print_node()
    print "Current Node : " , current_node.x_round, "," , current_node.y_round
    goal_node.print_node()
    print "Goal Node : " , goal_node.x_round, "," , goal_node.y_round'''
    for y in range(height):
        for x in range(width):
            if(current_node.x_round == x and current_node.y_round == y):
                if(current_node.x_round == goal_node.x_round and current_node.y_round == goal_node.y_round):
                    map_str += 'O'
                else:
                    map_str += '+'
            elif(goal_node.x_round == x and goal_node.y_round == y):
                if(current_node.x_round == goal_node.x_round and current_node.y_round == goal_node.y_round):
                    map_str += 'O'
                else:
                    map_str += 'X'
            else:  
                map_str += l_map[x][y]
        map_str +='\n'
    if(debug):
        print map_str

def print_path_to_solution(l_map, result_node, goal_node):
    map_str = ""
    width, height = get_map_size(l_map)
    
    #Populate a list with the path taken 
    path = []
    previous_node = result_node.came_from
    path.append(previous_node)
    if(debug):
        print "Printing Path"
    while previous_node != None:
        if(debug):
            print previous_node.print_node()
        previous_node = previous_node.came_from
        if(previous_node != None):
            path.append(previous_node)
    '''current_node.print_node()
    print "Current Node : " , current_node.x_round, "," , current_node.y_round
    goal_node.print_node()
    print "Goal Node : " , goal_node.x_round, "," , goal_node.y_round'''
    for y in range(height):
        for x in range(width):
            found = False
            for i in path:
                #print i
                if(i.x_round == x and i.y_round == y):
                     map_str += '*'
                     found = True
                     continue
            if(found):
                continue
            elif(result_node.x_round == x and result_node.y_round == y):
                if(result_node.x_round == goal_node.x_round and result_node.y_round == goal_node.y_round):
                    map_str += 'O'
                else:
                    map_str += '+'
            elif(goal_node.x_round == x and goal_node.y_round == y):
                if(result_node.x_round == goal_node.x_round and result_node.y_round == goal_node.y_round):
                    map_str += 'O'
                else:
                    map_str += 'X'
            else:  
                map_str += l_map[x][y]
        map_str +='\n'
    if(debug):
        print map_str

def print_list_to_solution(l_map, travel_list):
    map_str = ""
    width, height = get_map_size(l_map)
    if(debug):
        print "Printing Greedy Path"
    '''current_node.print_node()
    print "Current Node : " , current_node.x_round, "," , current_node.y_round
    goal_node.print_node()
    print "Goal Node : " , goal_node.x_round, "," , goal_node.y_round'''
    if(debug):
        print travel_list
    for y in range(height):
        for x in range(width):
            found = False
            for i in travel_list:
                #print i
                if(int(round(i[0])) == x and int(round(i[1])) == y):
                     if l_map[int(round(i[0]))][int(round(i[1]))] == "B":
                         map_str += "X"
                     else:
                         map_str += '*'
                     found = True
                     continue
            if(found):
                continue
            
            else:  
                map_str += l_map[x][y]
        map_str +='\n'
    if(debug):
        print map_str

def print_cost_in_map(l_map, nodes):
    map_str = ""
    width, height = get_map_size(l_map)
    
    '''current_node.print_node()
    print "Current Node : " , current_node.x_round, "," , current_node.y_round
    goal_node.print_node()
    print "Goal Node : " , goal_node.x_round, "," , goal_node.y_round'''
    for y in range(height):
        for x in range(width):
            map_str += l_map[x][y]
        map_str +='\n'
    if(debug):
        print map_str
        
    cost_map_str = ""

    max_length = int(math.ceil(math.log10(max_cost))) + 1 
    for y in range(height):
        for x in range(width):
            n_id = get_id([x,y], width, height)
            if (nodes.has_key(n_id)):
                if(nodes[n_id].cost >0):
                    append = ' ' * int(max_length - math.floor(math.log10(nodes[n_id].cost)) + 1)
                else:
                    append = ' ' * (max_length + 1)
                cost_map_str += append + str(nodes[n_id].cost) + " "
            else:
                cost_map_str += ' ' * (max_length+1) + '- '
            
        cost_map_str +='\n'
    if(debug):
        print cost_map_str

    f_cost_map_str = ""

    max_length = int(math.ceil(math.log10(max_cost))) + 1 
    for y in range(height):
        for x in range(width):
            n_id = get_id([x,y], width, height)
            if (nodes.has_key(n_id)):
                if(nodes[n_id].f_cost >0):
                    append = ' ' * int(max_length - math.floor(math.log10(nodes[n_id].f_cost)) + 1)
                else:
                    append = ' ' * (max_length + 1)
                f_cost_map_str += append + str(int(round(nodes[n_id].f_cost))) + " "
            else:
                f_cost_map_str += ' ' * (max_length+1) + '- '
            
        f_cost_map_str +='\n'
    if(debug):
        print f_cost_map_str

def score_result(length, buildings, roads, no_points, total_smoothness, map_width, map_height):
    k_i = 1.0 #initial score 
    k_l = 1.0 #path length
    k_b = 30.0 #buildings - very bad 
    k_r = 0.8 #roads 
    k_n = 0.2 # no of waypoints 
    k_s = 2.0 #smoothness

    score = max(0, k_i * (map_width + map_height) - k_l * length - k_b * buildings + k_r * roads - k_n * no_points - k_s * total_smoothness)

    return score

#def get_point_at(path, dist):
 
def get_dist(point1, point2):
    return math.hypot(point1[0] - point2[0], point1[1] - point2[1])

def get_point_at(point1, point2, dist):
    #get point dist from point2 towards point1 
    #m = double(point1[1] - point2[1]) / double(point1[0] - point2[0])
    dist_p = get_dist(point1, point2)
    ratio = dist / dist_p
    dx = point1[0] - point2[0]
    dy = point1[1] - point2[1]
    point3 = [point2[0] + dx * ratio, point2[1] + dy * ratio]
    if(debug):
        print point1
        print point2
        print point3
    return point3

def score_path(path, l_map):
    if(debug):
        print path 
    width = len(l_map)
    height = len(l_map[0])
    no_points = len(path)
    if(debug):
        print "Width " , width , " Height " , height , " No points " , no_points 
    
    #Now travel through the path and count the number of buildings, roads and smoothness 
    smoothness = 0

    for i in range(2,len(path)-1):
        if(debug):
            print "Considering Point Index : " , i 
        #get 1 back 
        j = i-1
        dist = 0

        point_back = None

        while(j >=0):
            temp_d = get_dist(path[j], path[j+1])
            if(debug):
                print "J " , j, " Temp Dist " , temp_d 
            if temp_d + dist >=1:
                if(debug):
                    print "Found path inside point"                
                point_back = get_point_at(path[j], path[j+1], 1-dist)
                break
            else:
                if(debug):
                    print "Point outside" 
                if(j>0):
                    dist += temp_d
                    j = j-1
                else:
                    if(debug):
                        print "Point outside but no more points - Projecting"
                    point_back = get_point_at(path[j], path[j+1], 1-dist)
                    break

        ### Get forward 
        j = i+1
        dist = 0

        point_forward = None

        while(j < len(path)):
            temp_d = get_dist(path[j], path[j-1])
            if(debug):
                print "J " , j, " Temp Dist " , temp_d 
            if temp_d + dist >=1:
                if(debug):
                    print "Found path inside point"                
                point_forward = get_point_at(path[j], path[j-1], 1-dist)
                break
            else:
                if(debug):
                    print "Point outside" 
                if(j< len(path)-1):
                    dist += temp_d
                    j = j+1
                else:
                    if(debug):
                        print "Point outside but no more points - Projecting"
                    point_forward = get_point_at(path[j], path[j-1], 1-dist)
                    break
        
        if(point_back == None):
            if(debug):
                print "Error - no back point"
            
        elif(point_forward == None):
            if(debug):
                print "Error - no forward point" 

        else:
            if(debug):
                print "Point Forward " , point_forward[0], "," , point_forward[1]
                print "Point Back    " , point_back[0], "," , point_back[1]

            smoothness += delta(point_back, path[i], point_forward)

    if(debug):
        print "Smoothness : " , smoothness 



def cost_to_go(current, goal):
    #use manhatton dist??
    return math.fabs(goal[0] - current[0]) + math.fabs(goal[1] - current[1])
    #math.hypot(goal[0] - current[0], goal[1] - current[1])

def get_best_ind(open_set, goal):
    best_ind = -1
    best_cost = max_cost * 2
    for i in range(len(open_set)):
        temp_c = open_set[i][1] + cost_to_go(open_set[i][0], goal)
        if(temp_c < best_cost):
            best_ind = i
            best_cost = temp_c
            
    if(debug):
        print "Best Ind : ", best_ind, " Proj Cost : ", best_cost
    return best_ind

def get_neighbours(current_pos, l_map):
    current_node = current_pos[0]
    current_cost = current_pos[1]
    deltax = [-1,0,1]
    deltay = [-1,0,1]
    neighbour_list = []
    for dx in deltax:
        for dy in deltay:
            if(dx == 0 and dy == 0):
                continue
            neighbour_node = [current_node[0] + dx, current_node[1] + dy]
            #For now we will assume that the buildings are obstacles 
            n_node_id = [int(round(neighbour_node[0])), int(round(neighbour_node[1]))]
            val = l_map[n_node_id[0], n_node_id[1]]
            if(val == 'B'):
                if(debug):
                    print "Building - treating as obstacle"
                neighbour_list.append([neighbour_node, current_cost + 1.0])#0.2])
                
            elif(val == '.'):
                if(debug):
                    print "Road"
                neighbour_list.append([neighbour_node, current_cost + 0.1])#0.2])

            elif(val == ' '):
                print "Normal"
                neighbour_list.append([neighbour_node, current_cost + 0.3])#1])
                
    if(debug):
        print neighbour_list
    return neighbour_list

        
def is_same_point(current, goal):
    if(round(current[0]) == round(goal[0]) and round(current[1]) == round(goal[1])):
        return True
    else:
        return False

def is_in_set(point, closed_set):
    for i in closed_set:
        #check if they are the same 
        #if(point[0] == 
        if(is_same_point(point, closed_set[0])):
            return True
    return False

#Lets assume that the map is discretized 
#so each state has a fixed id 

def get_id(point, width, height):
    return int(width * round(point[1]) + round(point[0]))

class Node():
    def __init__(self, n_id, x,y, cost, f_cost, came_from = None):
        self.id = n_id
        self.x = x
        self.y = y
        self.x_round = int(round(x))
        self.y_round = int(round(y))
        self.cost = cost
        self.f_cost = f_cost
        self.came_from = came_from

    def print_node(self):
        if(debug):
            print "ID : " , self.id , " Pos : " , self.x , "," , self.y

def get_best_node_from_set(open_set, nodes, goal):
    best_node = None
    best_cost = max_cost
    
    for i in open_set:
        temp_c = nodes[i].cost + cost_to_go([nodes[i].x, nodes[i].y], [goal.x, goal.y])
        if(temp_c < best_cost):
            best_node = nodes[i]
            best_cost = temp_c
    
    if(best_node == None):
        if(debug):
            print "Error : Best node not found"
        return None
    else:
        if(debug):
            print "Best Ind : ", best_node.id, " Proj Cost : ", best_cost
        return best_node

def add_and_get_neighbours(current_node, nodes, l_map):
    current_cost = current_node.cost
    deltax = [-1,0,1]
    deltay = [-1,0,1]

    neighbour_list = []
    width, height = get_map_size(l_map)
     
    for dx in deltax:
        for dy in deltay:
            if(dx == 0 and dy == 0):
                continue
            neighbour_pos = [current_node.x + dx, current_node.y + dy]
            
            n_id = get_id(neighbour_pos, width, height)
            
            n_node = None
            if(nodes.has_key(n_id)):
                n_node = nodes[n_id]
            else:
                n_node = Node(n_id, neighbour_pos[0], neighbour_pos[1], max_cost, max_cost, None)
                nodes[n_node.id] = n_node
            #For now we will assume that the buildings are obstacles 
            n_node_id = [int(round(neighbour_pos[0])), int(round(neighbour_pos[1]))]
            try: 
                val = l_map[n_node_id[0]][n_node_id[1]]
            except:
                print n_node_id

            if(val == 'B'):
                if(debug):
                    print "Building - treating as obstacle"
                neighbour_list.append([n_node, cost_building])
            elif(val == '#'):
                if(debug):
                    print "Building - treating as obstacle"               
                neighbour_list.append([n_node, cost_wall])
            elif(val == '.'):
                if(debug):
                    print "Road"
                neighbour_list.append([n_node, cost_road])#0.2])

            elif(val == ' '):
                if(debug):
                    print "Normal"
                neighbour_list.append([n_node, cost_normal]) #1])


    return neighbour_list

def is_in_same_set(c_id, check_set):
    return check_set.intersection([c_id])

def a_star(start, end, l_map):    
    #g_score - cost from the start 
    g_score = 0
    #f_score - cost to current position + estimated cost to go 
    
    f_score = g_score + cost_to_go(start, end)

    width, height = get_map_size(l_map)

    nodes = {}

    start_node = Node(get_id(start, width, height), start[0], start[1], 0, f_score, None)
    goal_node = Node(get_id(end, width, height), end[0], end[1], max_cost, 0, None)
    
    nodes[start_node.id] = start_node
    nodes[goal_node.id] = goal_node

    closed_set = set()
    open_set = set()

    open_set.add(start_node.id)

    if(debug):
        print "Goal Node"
    goal_node.print_node()
    
    while len(open_set) >0:
        current_node = get_best_node_from_set(open_set, nodes, goal_node)
        print_pos_in_map(l_map, current_node, goal_node)
        if(debug):
            print "Current Node"
            current_node.print_node()

        #If this happens the node objects should be the same 
        if(current_node.id == goal_node.id):
            if(debug):
                print "We are at goal - Done"
            path_to_goal = []
            path_to_goal.append([current_node.x, current_node.y])
            previous_node = current_node.came_from
            path_to_goal.append([previous_node.x, previous_node.y])
            if(debug):
                print "Printing Path"
            while previous_node != None:
                previous_node.print_node()
                previous_node = previous_node.came_from
                if previous_node != None:
                    path_to_goal.append([previous_node.x, previous_node.y])
            #print_path_to_solution(l_map, current_node, goal_node)
            return path_to_goal
            #break
        
        else:
            if(debug):
                print "Not at goal - exploring"

            # 
            if express_flag:
                express = check_express( [ current_node.x, current_node.y ], [ goal_node.x, goal_node.y ], l_map )
                if express:
                    if(debug):
                        print "We are at goal - Done"
                    path_to_goal = []
                    path_to_goal.append([goal_node.x, goal_node.y])
                    previous_node = current_node
                    path_to_goal.append([previous_node.x, previous_node.y])
                    if(debug):
                        print "Printing Path"
                    while previous_node != None:
                        previous_node.print_node()
                        previous_node = previous_node.came_from
                        if previous_node != None:
                            path_to_goal.append([previous_node.x, previous_node.y])
                    #print_path_to_solution(l_map, current_node, goal_node)
                    return path_to_goal

            else:
                express = False

            if express == False:

                closed_set.add(current_node.id)
                neighbour_list = add_and_get_neighbours(current_node, nodes, l_map)
                #neighbour_list.append( goal_node )
                if(debug):
                    print neighbour_list
                #remove current from the openset 
                open_set.remove(current_node.id)            #and add to closed set
                closed_set.add(current_node.id)

                for neigh in neighbour_list:
                    #print_pos_in_map(l_map, neigh[0], goal_node)
                    #import ipdb
                    #ipdb.set_trace()
                    neigh[0].print_node()
                    if(is_in_same_set(neigh[0].id, closed_set)):
                        if(debug):
                            print " In Closed Set"
                        continue
                    #Cost to current node + cost to get to the neighbour through this route 
                    t_score = current_node.cost + neigh[1]

                    if(debug):
                        print " T Score : " , t_score, " Current Score : " , neigh[0].cost

                    if not is_in_same_set(neigh[0], open_set) or t_score <= neigh[0].cost:
                        if(debug):
                            print "Found Node with a better path - Checking and adding to open set"
                        neigh[0].came_from = current_node
                        neigh[0].cost = t_score
                        neigh[0].f_cost = neigh[0].cost + cost_to_go([neigh[0].x, neigh[0].y], end)

                        if(not is_in_same_set(neigh[0].id, open_set)):
                            open_set.add(neigh[0].id)

        print_cost_in_map(l_map, nodes)

def a_star_light(start, end, l_map):    
    g_score = 0
    
    f_score = g_score + cost_to_go(start, end)

    width, height = get_map_size(l_map)

    nodes = {}

    start_node = Node(get_id(start, width, height), start[0], start[1], 0, f_score, None)
    goal_node = Node(get_id(end, width, height), end[0], end[1], max_cost, 0, None)
    
    nodes[start_node.id] = start_node
    nodes[goal_node.id] = goal_node

    closed_set = set()
    open_set = set()

    open_set.add(start_node.id)

    goal_node.print_node()
    
    while len(open_set) >0:
        current_node = get_best_node_from_set(open_set, nodes, goal_node)

        current_node.print_node()

        #If this happens the node objects should be the same 
        if(current_node.id == goal_node.id):
            path_to_goal = []
            path_to_goal.append([current_node.x, current_node.y])
            previous_node = current_node.came_from
            path_to_goal.append([previous_node.x, previous_node.y])
            while previous_node != None:
                previous_node = previous_node.came_from
                if previous_node != None:
                    path_to_goal.append([previous_node.x, previous_node.y])
            return path_to_goal
        else:
            closed_set.add(current_node.id)
            neighbour_list = add_and_get_neighbours(current_node, nodes, l_map)
            
            #remove current from the openset 
            open_set.remove(current_node.id)
            #and add to closed set
            closed_set.add(current_node.id)

            for neigh in neighbour_list:
                neigh[0].print_node()
                if(is_in_same_set(neigh[0].id, closed_set)):
                    continue
                #Cost to current node + cost to get to the neighbour through this route 
                t_score = current_node.cost + neigh[1]

                if not is_in_same_set(neigh[0], open_set) or t_score <= neigh[0].cost:
                    neigh[0].came_from = current_node
                    neigh[0].cost = t_score
                    neigh[0].f_cost = neigh[0].cost + cost_to_go([neigh[0].x, neigh[0].y], end)
                    
                    if(not is_in_same_set(neigh[0].id, open_set)):
                        open_set.add(neigh[0].id)

    return None

def get_map_size(l_map):
    width = len(l_map)
    height = len(l_map[0])
    return [width, height]

def parse_map( map_file ):
    f = open( map_file, 'r' )
    tmp = f.read()
    # if len( tmp ) > 3000:
    #     return None
    # else:
    #     x = tmp.split('\n')

    x = tmp.split('\n')
    f.close()
    # construct the transposed map
    untransposed_map = [ i for i in x if len( i ) > 0 ]
    transposed_map = []
    for idx, i in enumerate( untransposed_map[0] ):
        column = [ j[idx] for j in untransposed_map ]
        transposed_map.append( column )

    #width = len(transposed_map)
    #height = len(transposed_map[0])
    #return [transposed_map, width, height]
    return transposed_map

def get_next_pos( int_current_pos, current_pos, int_end, end, map_list ):
    #assert int_current_pos == [ int( round(i,0)) for i in current_pos ]
    #assert int_end == [ int( round(i,0)) for i in end ]

    greedy_next = greedy( int_current_pos, int_end, map_list )

    if greedy_next == None:
        greedy_next = end

    return greedy_next

"""
Just a baseline check
"""
if __name__ == '__main__':
    map_file = sys.argv[1]
    map_list = parse_map( map_file )
 
    while True:
        t = sys.stdin.readline()
        [ int_start, int_end ] = get_start_end( t )
        if int_start == int_end:
            final_output = int_start + int_end
            string_output = " ".join( [ str(x) for x in final_output ]  )
            write_output(string_output)
        else: 
            [ start, end ] = modify_start_end( int_start, int_end )

            # import random
            # int_start = [ random.randint( 0,len(map_list)-1 ), random.randint( 0, len( map_list[0] )-1 ) ]
            # int_end = [ random.randint( 0,len(map_list)-1 ), random.randint( 0, len( map_list[0] )-1 ) ]
            # print int_start, int_end
            # start = int_start
            # end = int_end

            # A* path
            if(True):
                path_to_goal = a_star(start, end, map_list)#a_star(start, end, map_list)
                if(path_to_goal == None):
                    path_to_goal = [start, end]
                path_to_goal.reverse()
                print_list_to_solution(map_list, path_to_goal)
                string_output = " ".join( [ str( x[0] ) + " " + str( x[1] ) for x in path_to_goal ] )
                write_output(string_output)

            # William's greedy path
            else:
                travel_list = []
                travel_list.append( start )
                current_pos = start


                int_current_pos = [ int( round( i, 0 ) ) for i in current_pos ]

                while ( int_current_pos != int_end ):
                    if map_list != None:
                        next_pos = get_next_pos( int_current_pos, current_pos, int_end, end, map_list )
                        current_pos = next_pos
                        int_current_pos = [ int( round( i, 0 ) ) for i in current_pos ]
                        if int_current_pos != int_end:
                            travel_list.append( next_pos )
                        else:
                            break

                travel_list.append( end )
                print_list_to_solution(map_list, travel_list)

                string_output = " ".join( [ str( x[0] ) + " " + str( x[1] ) for x in travel_list ] )
                write_output(string_output)
