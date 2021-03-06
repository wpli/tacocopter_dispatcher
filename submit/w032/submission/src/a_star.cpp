#include <iostream>
#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>
#include <map>
#include <set>

using namespace std;

#define map_edge_val 3
#define map_building_val 2
#define map_road_val 1
#define map_normal_val 0
#define HIGH 10000

#define debug 0
//which gave 298k

#define edge_cost 6
#define building_cost 4
#define road_cost 0.0266//1/30
#define normal_cost 0.6//0.033333
#define add_val 0//0.499999
#define free_map 1

int map_width = 0, map_height = 0; 

typedef struct _map_t{
    int **values;
    int width;
    int height;
} map_t;

struct _node_t;
typedef _node_t node_t;

struct _node_t{
    int id;
    double x;
    double y;
    int x_r;
    int y_r;
    double g_cost;
    double f_cost;
    //node_t *came_from;
    int came_from;
    double cost_from_neightbour;
};// node_t;

typedef struct _node_list_t{
    node_t *nodes;
    int no_nodes;
} node_list_t; 

//map<int, node_t> node_map_t;

double cost_to_go(double current[2], double goal[2]){
  return ( fabs(current[0] - goal[0]) + fabs(current[1] - goal[1]) ) * edge_cost;
}

double cost_to_go(node_t current, node_t goal){
    return fabs(current.x - goal.x) + fabs(current.y - goal.y);
}

double round(double r) {
    return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}

int get_id(double x, double y, int width){
    return width * round(y) + round(x); 
}

int get_id(int x, int y, int width){
    return width * y + x; 
}

vector<node_t *> smooth_path(vector<node_t *> path){
    vector<node_t *> smooth_path_list;
    int path_size = path.size();
    if(path_size == 2){
        return path;
    }

    int current_idx = 1;
    smooth_path_list.push_back(path[0]);
    while (current_idx <= (path_size - 2)){
        node_t *c_node = smooth_path_list[smooth_path_list.size()-1];//path[current_idx];
        node_t *node_1 = path[current_idx];
        node_t *node_2 = path[current_idx+1];
        
        double c_pos[2] = {c_node->x, c_node->y};
        double pos_1[2] = {node_1->x, node_1->y};
        double pos_2[2] = {node_2->x, node_2->y};

        double angle1 = atan2(pos_1[1] - c_pos[1], pos_1[0] - c_pos[0]);
        double angle2 = atan2(pos_2[1] - pos_1[1], pos_2[0] - pos_1[0]);

        int d_c_p2 = pow(pos_2[0] - c_pos[0],2) + pow(pos_2[1] - c_pos[1],2);
        int d_c_p1 = pow(pos_1[0] - c_pos[0],2) + pow(pos_1[1] - c_pos[1],2);
        int d_p2_p1 = pow(pos_1[0] - pos_2[0],2) + pow(pos_1[1] - pos_2[1],2);
        //fprintf(stderr, "Total : %f - Partial ; %f + %f \n", d_c_p2, d_c_p1, d_p2_p1);
        //fprintf(stderr, "Total : %d - Partial : %d + %d \n", d_c_p2, d_c_p1, d_p2_p1);

        //if(d_c_p2 == (d_c_p1 + d_p2_p1)){
        if(angle1 == angle2){
            //fprintf(stdout, "Can ignore middle point\n");
            //smooth_path_list.push_back(path[current_idx+2]);
            if(current_idx == (path_size - 2)){
                smooth_path_list.push_back(path[current_idx+1]);
            }
            current_idx+=1;
        }
        else{
            smooth_path_list.push_back(path[current_idx]);
            //smooth_path_list.push_back(path[current_idx+1]);
            if(current_idx == (path_size - 2)){
                smooth_path_list.push_back(path[current_idx+1]);
            }
            current_idx+=1;
        }
    }
    /*for(int i=0; i < smooth_path_list.size(); i++){
        fprintf(stderr, "[%d] = > %f,%f\n", smooth_path_list[i]->id, 
                smooth_path_list[i]->x, smooth_path_list[i]->y);
                }*/

    return smooth_path_list;
}

int modify_start_end(double start[2], double end[2]){
    if (start[0] < end[0]){
        start[0] += add_val;
        end[0] -= add_val;
    }
    else if (start[0] > end[0]){
        start[0] -= add_val;
        end[0] += add_val;
    }

    if (start[1] < end[1]){
        start[1] += add_val;
        end[1] -= add_val;
    }
    else if(start[1] > end[1]){
        start[1] -= add_val;
        end[1] += add_val;
    }
    return 0;
}

node_t * create_node(double x, double y, int width){
    node_t *node = (node_t *) calloc(1, sizeof(node_t));
    node->x = x;
    node->y = y;
    node->x_r = (int) round(x);
    node->y_r = (int) round(y);
    node->id = get_id(node->x_r, node->y_r, width);
    node->g_cost = HIGH;
    node->f_cost = HIGH;
    node->came_from = -1;//NULL;
    node->cost_from_neightbour = 0;
    return node;
}

node_t * create_node(double x, double y, double g_cost, double f_cost, int width){
    node_t *node = (node_t *) calloc(1, sizeof(node_t));
    node->x = x;
    node->y = y;
    node->x_r = (int) round(x);
    node->y_r = (int) round(y);
    node->id = get_id(node->x_r, node->y_r, width);
    node->g_cost = g_cost;
    node->f_cost = f_cost;
    node->came_from = -1;//NULL;
    node->cost_from_neightbour = 0;
    return node;
}

void init_node(node_t *node, double x, double y, int width){
    node->x = x;
    node->y = y;
    node->x_r = round(x);
    node->y_r = round(y);
    node->id = get_id(node->x_r, node->y_r, width);
    node->g_cost = HIGH;
    node->f_cost = HIGH;
    node->came_from = -1;//NULL;
    node->cost_from_neightbour = 0;
}

void init_node(node_t *node, double x, double y, double g_cost, double f_cost, int width){
    node->x = x;
    node->y = y;
    node->x_r = round(x);
    node->y_r = round(y);
    node->id = get_id(node->x_r, node->y_r, width);
    node->g_cost = g_cost;
    node->f_cost = f_cost;
    node->came_from = -1;//NULL;
    node->cost_from_neightbour = 0;
}

int get_best_node_from_set(set<int> c_set, map<int, node_t> node_map, node_t goal_node){
    int best_node_id = -1;
    double best_cost = HIGH *2;

    for (set<int>::iterator it=c_set.begin(); it!=c_set.end(); ++it){
        
        map<int,node_t>::iterator m_it;
        m_it = node_map.find(*it);
        if(m_it == node_map.end()){
            if(debug)
                fprintf(stdout, "Error - No element found\n");
            continue;
        }
        node_t c_node = m_it->second;
        
        
        double c_cost = c_node.g_cost + cost_to_go(c_node, goal_node);
        if(debug)
            fprintf(stdout, "Current Id : %d => Cost : %f\n", *it, c_cost);
        if(c_cost < best_cost){
            best_node_id = c_node.id;
            best_cost = c_cost;
        }
    }
    return best_node_id;
}

int get_best_node_from_set(set<int> c_set, map<int, node_t*> node_map, node_t goal_node){
    int best_node_id = -1;
    double best_cost = HIGH *2;

    for (set<int>::iterator it=c_set.begin(); it!=c_set.end(); ++it){
        
        map<int,node_t*>::iterator m_it;
        m_it = node_map.find(*it);
        if(m_it == node_map.end()){
            if(debug)
                fprintf(stdout, "Error - No element found\n");
            continue;
        }
        node_t c_node = *m_it->second;
        
        
        double c_cost = c_node.g_cost + cost_to_go(c_node, goal_node);
        if(debug)
            fprintf(stdout, "Current Id : %d => Cost : %f\n", *it, c_cost);
        if(c_cost < best_cost){
            best_node_id = c_node.id;
            best_cost = c_cost;
        }
    }
    return best_node_id;
}

//this map should be fine?? 
vector<node_t> get_neighbours(node_t c_node, map <int, node_t> *nodes, map_t *g_map){
    vector<node_t> n_list;
    
    double curr_cost = c_node.g_cost;
    //int deltax[3] = {-1, 0, 1};
    //int deltay[3] = {-1,0,-1};
    map<int,node_t>::iterator m_it;
    
    //fprintf(stdout, "Current Node [%d] Pos : %f,%f\n", c_node.id, c_node.x, c_node.y);

    for(int dx = -1; dx <=1; dx++){
        for(int dy = -1; dy <=1; dy++){
            if(dx == 0 && dy == 0)
                continue;
            
            double n_pos[2] = {c_node.x + dx, c_node.y+dy};

            int n_id = get_id(n_pos[0], n_pos[1], map_width);

            //fprintf(stdout, "N Node [%d] Pos : %f,%f\n", n_id, n_pos[0], n_pos[1]);

            m_it = nodes->find(n_id);
            node_t *n_node = NULL;

            if(m_it != nodes->end()){
                fprintf(stdout, "Already in Nodes\n");
                n_node = &m_it->second;
            }
            else{
                n_node = create_node(n_pos[0], n_pos[1], HIGH, HIGH, map_width);       
                nodes->insert(pair<int, node_t>(n_node->id, *n_node));
            }
            switch(g_map->values[n_node->x_r][n_node->y_r]){
	      /* 
	    case(map_edge_val):
                n_node->cost_from_neightbour = edge_cost;
                break;
	      */
                
            case(map_building_val):
                n_node->cost_from_neightbour = building_cost;
                break;
                
            case(map_road_val):
                n_node->cost_from_neightbour = road_cost;
                break;
                
            case(map_normal_val):
                n_node->cost_from_neightbour = normal_cost;
                break;
            }
            n_list.push_back(*n_node);
            //hmm - what happens to these nodes once the function is done?? 
            //if(nodes.
            //init_node(
        }
    }  
    

    return n_list;
}

int solve(map_t *g_map, double start_xy[2], double goal_xy[2]){//node_t *start, node_t *end){
    int width = g_map->width;
    int height = g_map->height;

    if(width == 0 || height == 0){
        fprintf(stdout, "Map size error : %d,%d ", width, height);
        return -4;
    }

    if(debug)
        fprintf(stdout, "Start : %.2f, %.2f => Goal : %.2f, %.2f   ", start_xy[0], start_xy[1], goal_xy[0], goal_xy[1]);

    if(round(start_xy[0]) <0 || round(start_xy[1] < 0)){
        return -1;
    }

    if(round(goal_xy[0]) <0 || round(goal_xy[1] < 0)){
        return -1;
    }
    
    //vector<node_t> nodes;
    map <int, node_t*> nodes;

    double f_cost_start = cost_to_go(start_xy, goal_xy);
    node_t *start_node = create_node(start_xy[0], start_xy[1], 0, f_cost_start, width);

    node_t *goal_node = create_node(goal_xy[0], goal_xy[1], HIGH, 0, width);

    nodes.insert(pair<int, node_t*>(start_node->id, start_node));
    nodes.insert(pair<int, node_t*>(goal_node->id, goal_node));

    set<int> closed_set;
    set<int> open_set;
    set<int>::iterator set_it;
    map<int,node_t*>::iterator m_it;

    open_set.insert(start_node->id);
    
    while(open_set.size() > 0){
        int best_node_id = get_best_node_from_set(open_set, nodes, *goal_node);
        if(best_node_id <0){
            if(debug)
                fprintf(stdout, "Error - No best node in open set\n");
            if(free_map){
                for(m_it = nodes.begin(); m_it != nodes.end(); ++m_it){
                    free(m_it->second);
                }
            }
            return -1;
        }
        if(debug)
            fprintf(stdout, "Best Node in Open Set : %d\n", best_node_id);
        
        m_it = nodes.find(best_node_id);
        node_t best_node = *m_it->second;

        if(best_node_id == goal_node->id){
            if(debug){
                fprintf(stdout, "Sucess - Found Goal node\n");
                fprintf(stdout, "Path to Goal\n\n");
                fprintf(stdout, "Goal : %d - %f,%f\n", goal_node->id, goal_node->x, goal_node->y);
            }
            
            vector<node_t *> path; 
            path.insert(path.begin(),goal_node);
            int next_id = goal_node->came_from; 
            while(next_id >=0){
                m_it = nodes.find(next_id);
                if(m_it != nodes.end()){
                    node_t next_node = *m_it->second;
                    path.insert(path.begin(), m_it->second);//next_node);
                    next_id = next_node.came_from;
                    //fprintf(stdout, "Node : %d - %f,%f\n", next_node.id, next_node.x, next_node.y);
                }
                
            }
            if(path[0]->id != start_node->id){
                path.insert(path.begin(), start_node);
            }

            string output_str;
            char buf[100]; 
            int first = 1;

            //fprintf(stderr, "Goal ID %d - Start ID : %d\n", goal_node->id, start_node->id);            
            if(path.size() < 2){
                //fprintf(stdout, "Width : %d Height : %d Start : %.2f, %.2f (%d,%d) => Goal : %.2f, %.2f (%d,%d)  ", width, height, start_xy[0], start_xy[1], (int) round(start_xy[0]), (int) round(start_xy[1]), goal_xy[0], goal_xy[1], (int) round(goal_xy[0]), (int) round(goal_xy[1])); 
                //fprintf(stdout, "[%d - %d]", goal_node->id, start_node->id);      

                //for(int i=0; i < path.size(); i++){
                //  fprintf(stdout, "[%d] - %.2f %.2f ", path[i]->id, path[i]->x, path[i]->y);
                //}
                //fprintf(stdout,"\n");
                if(free_map){
                    for(m_it = nodes.begin(); m_it != nodes.end(); ++m_it){
                        free(m_it->second);
                    }
                }
                return -4;
            }

           vector<node_t *> smoothed_path = smooth_path(path);
            
            for(int i=0; i < smoothed_path.size(); i++){
                if(first){
                    sprintf(buf, "%f %f", smoothed_path[i]->x, smoothed_path[i]->y);
                    first = 0;
                }
                else{
                    sprintf(buf, " %f %f", smoothed_path[i]->x, smoothed_path[i]->y);
                }
                output_str.append(buf);
            }
            //fprintf(stdout, "%s\n", output_str.c_str());
            cout << output_str << endl;
            if(free_map){
                for(m_it = nodes.begin(); m_it != nodes.end(); ++m_it){
                    free(m_it->second);
                }
            }
            return 0;
        }
        else{ //do the serious work 
            closed_set.insert(best_node_id);
            open_set.erase(best_node_id);
            
            //Get neighbours 
            vector<node_t> n_node_list;
            //////////////////////////////////////////////////
            for(int dx = -1; dx <=1; dx++){
                for(int dy = -1; dy <=1; dy++){
                    if(dx == 0 && dy == 0)
                        continue;
            
                    double n_pos[2] = {best_node.x + dx, best_node.y+dy};

                    //we need to check if this goes outside the map 
                    if(round(n_pos[0]) <0 || round(n_pos[1] < 0)){
                        continue;
                    }

                    int n_id = get_id(n_pos[0], n_pos[1], width);

                    m_it = nodes.find(n_id);
                    node_t *n_node = NULL;

                    if(m_it != nodes.end()){
                        if(debug)
                            fprintf(stdout, "Already in Nodes\n");
                        n_node = m_it->second;
                    }
                    else{
                        n_node = create_node(n_pos[0], n_pos[1], HIGH, HIGH, width);       
                        nodes.insert(pair<int, node_t*>(n_node->id, n_node));
                    }

                    if((n_node->x_r <0 || n_node->x_r > map_width-1) ||
                       (n_node->y_r <0 || n_node->y_r > map_height-1)){
                        n_node->cost_from_neightbour = HIGH;
                        continue;
                    }                    
                         
                    switch(g_map->values[n_node->x_r][n_node->y_r]){
                    case(map_edge_val):
                        n_node->cost_from_neightbour = edge_cost;
                        break;
                
                    case(map_building_val):
                        n_node->cost_from_neightbour = building_cost;
                        break;
                
                    case(map_road_val):
                        n_node->cost_from_neightbour = road_cost;
                        break;
                
                    case(map_normal_val):
                        n_node->cost_from_neightbour = normal_cost;
                        break;
                    }
                    n_node_list.push_back(*n_node);
                    //hmm - what happens to these nodes once the function is done?? 
                }
            }  

            ///////////////////////////////////////////////////////////
            //vector<node_t> n_node_list = get_neighbours(best_node, &nodes, g_map);

            for(int i=0; i < n_node_list.size(); i++){
                m_it = nodes.find(n_node_list[i].id);
                node_t *n_node = m_it->second;
                if(debug)
                    fprintf(stdout, "N Node [%d] Pos : %f,%f\n", n_node->id, n_node->x, n_node->y);
                //now lets do stuff with the neighbour nodes 
                
                set_it = closed_set.find(n_node->id);
                if(set_it != closed_set.end()){
                    if(debug)
                        fprintf(stdout, "Neighbour in closed set - continuing\n");
                    continue;
                }

                set_it = open_set.find(n_node->id);
                int not_in_open_set = 1;
                if(set_it != open_set.end()){
                    if(debug)
                        fprintf(stdout, "Neighbour in open set - processing\n");
                    not_in_open_set = 0;
                }
                
                double t_score = best_node.g_cost + n_node->cost_from_neightbour;

                if(debug)
                    fprintf(stdout, "N ID: %d - T Cost : %f, Current Cost : %f\n", n_node->id, t_score, n_node->g_cost);
                
                if(not_in_open_set == 1 || t_score <= n_node->g_cost){
                    //fprintf(stdout, "Found a better path : %d\n", n_node->id);
                    n_node->g_cost = t_score;
                    //fprintf(stdout, "New Cost : %f\n", n_node->g_cost);
                    n_node->f_cost = n_node->g_cost + cost_to_go(*n_node, *goal_node);
                    n_node->came_from = best_node.id;
                    if(not_in_open_set == 1){
                        open_set.insert(n_node->id);
                    }
                }
            }    
        }
    }
    

    //remember to free these guys 
    if(free_map){
        for(m_it = nodes.begin(); m_it != nodes.end(); ++m_it){
            free(m_it->second);
        }
    }

    if(debug)
        fprintf(stdout, "Failed to find Solution\n");

    return -1;
}

        /*
node_list_t * solve_1(map_t *g_map, double start_xy[2], double goal_xy[2]){//node_t *start, node_t *end){
    fprintf(stdout, "Start : %.2f, %.2f => Goal : %.2f, %.2f\n", start_xy[0], start_xy[1], goal_xy[0], goal_xy[1]);

    //vector<node_t> nodes;
    map <int, node_t> nodes;

    node_t start_node;
    double f_cost_start = cost_to_go(start_xy, goal_xy);
    init_node(&start_node, start_xy[0], start_xy[1], 0, f_cost_start);

    node_t goal_node;
    init_node(&goal_node, goal_xy[0], goal_xy[1], HIGH, 0);

    nodes.insert(pair<int, node_t>(start_node.id, start_node));
    nodes.insert(pair<int, node_t>(goal_node.id, goal_node));

    set<int> closed_set;
    set<int> open_set;
    
    open_set.insert(start_node.id);
    
    while(open_set.size() > 0){
        int best_node_id = get_best_node_from_set(open_set, nodes, goal_node);
        if(best_node_id <0){
            fprintf(stdout, "Error - No best node in open set\n");
            return NULL;
        }
        fprintf(stdout, "Best Node in Open Set : %d\n", best_node_id);
        map<int,node_t>::iterator m_it;
        m_it = nodes.find(best_node_id);
        node_t best_node = m_it->second;

        if(best_node_id == goal_node.id){
            fprintf(stdout, "Sucess - Found Goal node\n");
            //should return the path - not NULL
            return NULL;
        }
        else{ //do the serious work 
            closed_set.insert(best_node_id);
            open_set.erase(best_node_id);
            vector<node_t> n_nodes = get_neighbours(best_node, nodes, g_map);
        }        
    }

    fprintf(stdout, "Failed to find Solution\n");
    return NULL;
    }*/

void print_map(map_t *map){
    fprintf(stdout, "Map size : %d, %d\n", map->width, map->height);
    for(int j=0; j< map->height; j++){
        for(int i=0; i < map->width; i++){
            switch(map->values[i][j]){
            case map_normal_val:
                fprintf(stdout, " "); 
                break;
            case map_road_val:
                fprintf(stdout, "."); 
                break;
            case map_building_val:
                fprintf(stdout, "B"); 
                break;
            case map_edge_val:
                fprintf(stdout, "#"); 
                break;
            }
        }
        fprintf(stdout, "\n");
    }
    fprintf(stdout, "\n");
}

int main(int argc, char *argv[])
{
    //fprintf(stdout, "%d\n", argc);
    if(argc <2)
        return -1; 

    //setlinebuf(stdout);
    if(debug){
        fprintf(stdout, "Map Path : %s\n", argv[1]);
    }
    
    ifstream map_file (argv[1]);
    vector<string> map_str; 

    int width = 0, height = 0;
    if (map_file.is_open()){
        while (map_file.good() ){
            string line;
            getline (map_file,line);
            if(line.length() == 0){
                break;
            }
            height++;
            //cout << line << endl;
            width = line.length();
            //string cp =  line.copy();//new string(line);
            map_str.push_back(line);
            //cout << "Length" << line.length() << endl;
        }
        map_file.close();
    }

    //fprintf(stdout, "Map Width : %d Height : %d\n", width, height);

    if(0){
        fprintf(stdout, "Printing Map cache\n");
        for(int i=0; i < map_str.size(); i++){
            cout << map_str[i] << endl;
        }
    }

    map_t map;

    map_width = width;
    map_height = height; 
    map.width = width;
    map.height = height;
    map.values =  (int **) calloc(map.width, sizeof(int *));

    for(int i=0; i < map.width; i++){
        map.values[i] = (int *) calloc(map.height, sizeof(int));
    }
    
    for(int j=0; j< map.height; j++){
        string map_line = map_str[j];
        for(int i=0; i < map.width; i++){
            if(map_line[i]==' ')
                map.values[i][j] = map_normal_val;
            if(map_line[i]=='.')
                map.values[i][j] = map_road_val;
            if(map_line[i]=='B')
                map.values[i][j] = map_building_val;
            if(map_line[i]=='#')
                map.values[i][j] = map_edge_val;
        }
    }

    if(0){
        print_map(&map);
    }

    int c_count = 0;
    while(1){
        string input;
        getline (cin, input);
        //cout << input<<endl;
        c_count++;
        unsigned found = input.find(' ');

        int count = 0;
        double start_and_goal[4] = {.0};
        int start = 0;
        while(found != string::npos && found < input.length()){
            string str = input.substr(start, (found - start +1));
            start_and_goal[count] = atof(str.c_str());
            count++;
            start = found +1;
            found = input.find(' ', found+1);
        }
        
        string str = input.substr(start, (found - start +1));
        start_and_goal[count] = atof(str.c_str());

        if(debug){
            fprintf(stdout, "Start : %f,%f => Goal %f, %f\n", start_and_goal[0], start_and_goal[1], start_and_goal[2], start_and_goal[3]);
        }

        modify_start_end(start_and_goal, &start_and_goal[2]);
        
        //time_t t, t1;
        //time(&t)
        //long double t = time(0);
        int result = solve(&map, start_and_goal, &start_and_goal[2]);

        //long double t1 = time(0);
        //double diff = t1-t;
        //fprintf(stderr, "Time gap %.4f", diff);

        /*if(result == -4){
            //just give direct line 
            //fprintf(stdout, "%.2f %.2f %.2f %.2f\n", start_and_goal[0], start_and_goal[1], start_and_goal[2], start_and_goal[3]);
            //fprintf(stdout, "%.2f %.2f %.2f %.2f %.2f %.2f\n", start_and_goal[0], start_and_goal[1], start_and_goal[0], start_and_goal[1], start_and_goal[2], start_and_goal[3]);
            fprintf(stdout, " No Times : %d\n", c_count);
        }

        else*/
        if(result != 0){
            //just give direct line 
            //fprintf(stdout, "%.2f %.2f %.2f %.2f\n", start_and_goal[0], start_and_goal[1], start_and_goal[2], start_and_goal[3]);
            fprintf(stdout, "%f %f %f %f\n", start_and_goal[0], start_and_goal[1], start_and_goal[2], start_and_goal[3]);
        }
    }

    if(debug)
        fprintf(stdout, "Map Size : Width : %d, Height : %d\n", width, height);
}
