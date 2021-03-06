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

#define edge_cost 1.0
#define building_cost 1.0
#define road_cost -0.02666//1/30
#define normal_cost 0.033333
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
    return fabs(current[0] - goal[0]) + fabs(current[1] - goal[1]);
}

double cost_to_go(node_t current, node_t goal){
    return fabs(current.x - goal.x) + fabs(current.y - goal.y);
}

double round(double r) {
    return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}

int get_id(double x, double y){
    return map_width * round(y) + round(x); 
}

int get_id(int x, int y){
    return map_width * y + x; 
}

node_t * create_node(double x, double y){
    node_t *node = (node_t *) calloc(1, sizeof(node_t));
    node->x = x;
    node->y = y;
    node->x_r = round(x);
    node->y_r = round(y);
    node->id = get_id(node->x_r, node->y_r);
    node->g_cost = HIGH;
    node->f_cost = HIGH;
    node->came_from = -1;//NULL;
    node->cost_from_neightbour = 0;
    return node;
}

node_t * create_node(double x, double y, double g_cost, double f_cost){
    node_t *node = (node_t *) calloc(1, sizeof(node_t));
    node->x = x;
    node->y = y;
    node->x_r = round(x);
    node->y_r = round(y);
    node->id = get_id(node->x_r, node->y_r);
    node->g_cost = g_cost;
    node->f_cost = f_cost;
    node->came_from = -1;//NULL;
    node->cost_from_neightbour = 0;
    return node;
}

void init_node(node_t *node, double x, double y){
    node->x = x;
    node->y = y;
    node->x_r = round(x);
    node->y_r = round(y);
    node->id = get_id(node->x_r, node->y_r);
    node->g_cost = HIGH;
    node->f_cost = HIGH;
    node->came_from = -1;//NULL;
    node->cost_from_neightbour = 0;
}

void init_node(node_t *node, double x, double y, double g_cost, double f_cost){
    node->x = x;
    node->y = y;
    node->x_r = round(x);
    node->y_r = round(y);
    node->id = get_id(node->x_r, node->y_r);
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
                fprintf(stderr, "Error - No element found\n");
            continue;
        }
        node_t c_node = m_it->second;
        
        
        double c_cost = c_node.g_cost + cost_to_go(c_node, goal_node);
        if(debug)
            fprintf(stderr, "Current Id : %d => Cost : %f\n", *it, c_cost);
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
                fprintf(stderr, "Error - No element found\n");
            continue;
        }
        node_t c_node = *m_it->second;
        
        
        double c_cost = c_node.g_cost + cost_to_go(c_node, goal_node);
        if(debug)
            fprintf(stderr, "Current Id : %d => Cost : %f\n", *it, c_cost);
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
    
    //fprintf(stderr, "Current Node [%d] Pos : %f,%f\n", c_node.id, c_node.x, c_node.y);

    for(int dx = -1; dx <=1; dx++){
        for(int dy = -1; dy <=1; dy++){
            if(dx == 0 && dy == 0)
                continue;
            
            double n_pos[2] = {c_node.x + dx, c_node.y+dy};

            int n_id = get_id(n_pos[0], n_pos[1]);

            //fprintf(stderr, "N Node [%d] Pos : %f,%f\n", n_id, n_pos[0], n_pos[1]);

            m_it = nodes->find(n_id);
            node_t *n_node = NULL;

            if(m_it != nodes->end()){
                fprintf(stderr, "Already in Nodes\n");
                n_node = &m_it->second;
            }
            else{
                n_node = create_node(n_pos[0], n_pos[1], HIGH, HIGH);       
                nodes->insert(pair<int, node_t>(n_node->id, *n_node));
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
            n_list.push_back(*n_node);
            //hmm - what happens to these nodes once the function is done?? 
            //if(nodes.
            //init_node(
        }
    }  
    

    return n_list;
}

int solve(map_t *g_map, double start_xy[2], double goal_xy[2]){//node_t *start, node_t *end){
    if(debug)
        fprintf(stderr, "Start : %.2f, %.2f => Goal : %.2f, %.2f\n", start_xy[0], start_xy[1], goal_xy[0], goal_xy[1]);

    if(round(start_xy[0]) <0 || round(start_xy[1] < 0)){
        return -1;
    }

    if(round(goal_xy[0]) <0 || round(goal_xy[1] < 0)){
        return -1;
    }
    
    //vector<node_t> nodes;
    map <int, node_t*> nodes;

    double f_cost_start = cost_to_go(start_xy, goal_xy);
    node_t *start_node = create_node(start_xy[0], start_xy[1], 0, f_cost_start);

    node_t *goal_node = create_node(goal_xy[0], goal_xy[1], HIGH, 0);

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
                fprintf(stderr, "Error - No best node in open set\n");
            for(m_it = nodes.begin(); m_it != nodes.end(); ++m_it){
                free(m_it->second);
            }
            return -1;
        }
        if(debug)
            fprintf(stderr, "Best Node in Open Set : %d\n", best_node_id);
        
        m_it = nodes.find(best_node_id);
        node_t best_node = *m_it->second;

        if(best_node_id == goal_node->id){
            if(debug){
                fprintf(stderr, "Sucess - Found Goal node\n");
                fprintf(stderr, "Path to Goal\n\n");
                fprintf(stderr, "Goal : %d - %f,%f\n", goal_node->id, goal_node->x, goal_node->y);
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
                    //fprintf(stderr, "Node : %d - %f,%f\n", next_node.id, next_node.x, next_node.y);
                }
            }
            for(int i=0; i < path.size(); i++){
                fprintf(stderr, "%f %f ", path[i]->x, path[i]->y);
            }
            fprintf(stderr, "\n");
            for(m_it = nodes.begin(); m_it != nodes.end(); ++m_it){
                free(m_it->second);
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

                    int n_id = get_id(n_pos[0], n_pos[1]);

                    m_it = nodes.find(n_id);
                    node_t *n_node = NULL;

                    if(m_it != nodes.end()){
                        if(debug)
                            fprintf(stderr, "Already in Nodes\n");
                        n_node = m_it->second;
                    }
                    else{
                        n_node = create_node(n_pos[0], n_pos[1], HIGH, HIGH);       
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
                    fprintf(stderr, "N Node [%d] Pos : %f,%f\n", n_node->id, n_node->x, n_node->y);
                //now lets do stuff with the neighbour nodes 
                
                set_it = closed_set.find(n_node->id);
                if(set_it != closed_set.end()){
                    if(debug)
                        fprintf(stderr, "Neighbour in closed set - continuing\n");
                    continue;
                }

                set_it = open_set.find(n_node->id);
                int not_in_open_set = 1;
                if(set_it != open_set.end()){
                    if(debug)
                        fprintf(stderr, "Neighbour in open set - processing\n");
                    not_in_open_set = 0;
                }
                
                double t_score = best_node.g_cost + n_node->cost_from_neightbour;

                if(debug)
                    fprintf(stderr, "N ID: %d - T Cost : %f, Current Cost : %f\n", n_node->id, t_score, n_node->g_cost);
                
                if(not_in_open_set == 1 || t_score <= n_node->g_cost){
                    //fprintf(stderr, "Found a better path : %d\n", n_node->id);
                    n_node->g_cost = t_score;
                    //fprintf(stderr, "New Cost : %f\n", n_node->g_cost);
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
    for(m_it = nodes.begin(); m_it != nodes.end(); ++m_it){
        free(m_it->second);
    }

    if(debug)
        fprintf(stderr, "Failed to find Solution\n");

    return -1;
}

        /*
node_list_t * solve_1(map_t *g_map, double start_xy[2], double goal_xy[2]){//node_t *start, node_t *end){
    fprintf(stderr, "Start : %.2f, %.2f => Goal : %.2f, %.2f\n", start_xy[0], start_xy[1], goal_xy[0], goal_xy[1]);

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
            fprintf(stderr, "Error - No best node in open set\n");
            return NULL;
        }
        fprintf(stderr, "Best Node in Open Set : %d\n", best_node_id);
        map<int,node_t>::iterator m_it;
        m_it = nodes.find(best_node_id);
        node_t best_node = m_it->second;

        if(best_node_id == goal_node.id){
            fprintf(stderr, "Sucess - Found Goal node\n");
            //should return the path - not NULL
            return NULL;
        }
        else{ //do the serious work 
            closed_set.insert(best_node_id);
            open_set.erase(best_node_id);
            vector<node_t> n_nodes = get_neighbours(best_node, nodes, g_map);
        }        
    }

    fprintf(stderr, "Failed to find Solution\n");
    return NULL;
    }*/

void print_map(map_t *map){
    for(int j=0; j< map->height; j++){
        for(int i=0; i < map->width; i++){
            switch(map->values[i][j]){
            case map_normal_val:
                fprintf(stderr, " "); 
                break;
            case map_road_val:
                fprintf(stderr, "."); 
                break;
            case map_building_val:
                fprintf(stderr, "B"); 
                break;
            case map_edge_val:
                fprintf(stderr, "#"); 
                break;
            }
        }
        fprintf(stderr, "\n");
    }
    fprintf(stderr, "\n");
}

int main(int argc, char *argv[])
{
    setlinebuf(stdout);
    if(debug){
        fprintf(stderr, "Map Path : %s\n", argv[1]);
    }
    
    ifstream map_file (argv[1]);
    vector<string> map_str; 

    int width = 0, height = 0;
    if (map_file.is_open()){
        while (map_file.good() ){
            string line;
            height++;
            getline (map_file,line);
            //cout << line << endl;
            width = line.length();
            //string cp =  line.copy();//new string(line);
            map_str.push_back(line);
            //cout << "Length" << line.length() << endl;
        }
        map_file.close();
    }

    if(debug){
        fprintf(stderr, "Printing Map cache\n");
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

    if(debug){
        print_map(&map);
    }

    while(1){
        string input;
        getline (cin, input);
        //cout << input<<endl;
        
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
            fprintf(stderr, "Start : %.2f,%.2f => Goal %.2f, %.2f\n", start_and_goal[0], start_and_goal[1], start_and_goal[2], start_and_goal[3]);
        }
        
        int result = solve(&map, start_and_goal, &start_and_goal[2]);
        if(result != 0){
            //just give direct line 
            //fprintf(stderr, "%.2f %.2f %.2f %.2f\n", start_and_goal[0], start_and_goal[1], start_and_goal[2], start_and_goal[3]);
            fprintf(stderr, "%f %f %f %f\n", start_and_goal[0], start_and_goal[1], start_and_goal[2], start_and_goal[3]);
        }
    }

    if(debug)
        fprintf(stderr, "Map Size : Width : %d, Height : %d\n", width, height);
}
