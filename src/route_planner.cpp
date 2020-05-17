#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Store the closest nodes to the starting and ending coordinates in the RoutePlanner's start_node and end_node attributes.
    start_node = &(m_Model.FindClosestNode(start_x, start_y));
    end_node = &(m_Model.FindClosestNode(end_x, end_y));
}

// function to determine Hvalue of a node which is given by its distance from end node
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance((*end_node));
}

// method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
    (*current_node).FindNeighbors();
    // - For each node in current_node.neighbors,
    for(RouteModel::Node *node: current_node->neighbors){
        //set the parent, the h_value, the g_value, add to the open list and mark visited
        node->parent = current_node;
        node->h_value = CalculateHValue(node);
        node->g_value = current_node->g_value+current_node->distance(*node);
        open_list.emplace_back(node);
        node->visited = true;
    }
}

// method to sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
    // Sort the open_list according to the sum of the h value and g value in descending order by using lambda function for key.
    std::sort(open_list.begin(), open_list.end(), [](RouteModel::Node* const &a, RouteModel::Node* const &b) {
        return (a->g_value+a->h_value) > (b->g_value+b->h_value); 
    });
    // return the last node's pointer after removing it from the list
    RouteModel::Node* next_node = open_list.back();
    open_list.pop_back();
    return next_node;
}

// method to return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // iteratively follow the chain of parents of nodes until the starting node is found.
    while (current_node!=start_node)
    {
        // add the node to path and add the distance from the node to its parent to the distance variable.
        path_found.push_back(*current_node);
        distance+=current_node->distance(*(current_node->parent));
        current_node = current_node->parent;
    }    
    path_found.push_back(*current_node); // push the remaining starting node to the path
    std::reverse(path_found.begin(), path_found.end()); //reverse the path to get the currect order from start to end
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

// A* Search algorithm
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    // push the start node into the openlist and mark it visited
    open_list.push_back(start_node);
    start_node->visited=true;
    while(!open_list.empty()){
        current_node = NextNode();
        if(current_node==end_node){
            m_Model.path = ConstructFinalPath(current_node);
            break;
        }
        AddNeighbors(current_node);
    }
}