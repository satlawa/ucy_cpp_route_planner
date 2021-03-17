#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // calculate straigt distance to end node to use it as th h value
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value.
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // use FindNeighbors method to get all the neighbors
    current_node->FindNeighbors();

    // loop through all the neighbors of current node
    for (RouteModel::Node* neighbor : current_node->neighbors) {
        // set parent
        neighbor->parent = current_node;
        // calculate and set h value
        neighbor->h_value = CalculateHValue(neighbor);
        // calculate and set g value
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        // add neighbor to open_list
        open_list.push_back(neighbor);
        // set visited to true
        neighbor->visited = true;
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    // sort the vector according to the sum of the h value and g value
    std::sort(open_list.begin(), open_list.end(), [](const auto &x, const auto &y){
      return x->h_value + x->g_value > y->h_value + y->g_value;
    });
    // capture the vectors last value
    RouteModel::Node* next_node = open_list.back();
    // delete the vectors last value
    open_list.pop_back();
    return next_node;

}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.

    // loop through all nodes
    while (current_node->parent != nullptr) {
        // add node to path_found vector
        path_found.push_back(*current_node);
        // add distance (current node - parent) to overall distance
        distance += current_node->distance(*(current_node->parent));
        // set current_node to its parent
        current_node = current_node->parent;
    }
    // add start node
    path_found.push_back(*current_node);
    // reverse the elements in the vector
    std::reverse(path_found.begin(), path_found.end());
    // Multiply the distance by the scale of the map to get meters.
    distance *= m_Model.MetricScale();
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.

    // set start note as visited
    start_node->visited = true;
    // add startnode to open list vector
    open_list.push_back(start_node);

    // loop as long as ther are nodes left
    while(open_list.size() > 0) {
        // obtain next node
        current_node = NextNode();
        // if distance from current node to end node is 0
        if (current_node->distance(*end_node) == 0) {
            // create the final path
            m_Model.path = ConstructFinalPath(current_node);
            // end search
            return;
        }
        // add neighbors of current node
        AddNeighbors(current_node);
    }
}
