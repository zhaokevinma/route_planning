#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    RoutePlanner::start_node = &RoutePlanner::m_Model.FindClosestNode(start_x, start_y);
    RoutePlanner::end_node = &RoutePlanner::m_Model.FindClosestNode(end_x, end_y);

}


// Implement the CalculateHValue method.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*RoutePlanner::end_node);
}


// Expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
    current_node->FindNeighbors();
    // For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
    for (auto neighbor : current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->h_value = CalculateHValue(neighbor);
        // Add the neighbor to open_list and set the node's visited attribute to true.
        open_list.push_back(neighbor);
        neighbor->visited = true;
    }
}


// Sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
    // Sort the open_list according to the sum of the h value and g value.
    std::sort(open_list.begin(), open_list.end(), [](const auto *node1, const auto *node2){
        return (node1->h_value + node1->g_value) < (node2->h_value + node2->g_value);
    });
    // Create a pointer to the node in the list with the lowest sum.
    RouteModel::Node* lowest_node = open_list.front();
    // Remove that node from the open_list.
    open_list.erase(open_list.begin());
    // Return the pointer.
    return lowest_node;
}


// Return the final path found from A* search.
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    // For each node in the chain, add the distance from the node to its parent to the distance variable.
    while(current_node->parent != nullptr) {
        path_found.push_back(*current_node);
        distance += current_node->distance(*(current_node->parent));
        current_node = current_node->parent;
    }
    path_found.push_back(*current_node);
    // Reverse the vector because start node should be the first while end should be the last
    reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


//  A* Search algorithm.

void RoutePlanner::AStarSearch() {
    start_node->visited = true;
    open_list.push_back(start_node);
    RouteModel::Node *current_node = nullptr;

    while(!open_list.empty()) {
        // Use the NextNode() method to sort the open_list and return the next node.
        RouteModel::Node *current_node = NextNode(); 
        // When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
        if (current_node->distance(*end_node) == 0) {
            // Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        // Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
        AddNeighbors(current_node);
    }
}