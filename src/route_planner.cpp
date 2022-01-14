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
    start_node = &model.FindClosestNode(start_x, start_y);
    end_node = &model.FindClosestNode(end_x, end_y);

}


//   Implement the CalculateHValue method.
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddToOpen(RouteModel::Node *node){
    //add the node to open_list
    
    open_list.push_back(node);

    //set the node's visited attribute to true. Equivalent to State::kClosed;
    node->visited = true;
}

// Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
    current_node->FindNeighbors();

    // For each node in current_node.neighbors, set the parent, the h_value, the g_value.
    for (auto *neighbor : current_node->neighbors){
        // set the parent
        neighbor->parent = current_node;

        // set g_value
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);

        // set h_value =
        neighbor->h_value = CalculateHValue(neighbor);
        
        // For each node in current_node.neighbors, add the neighbor to open_list, equivalent to AddToOpen()
        //open_list.push_back(neighbor);

        //set the node's visited attribute to true. Equivalent to State::kClosed;
        //neighbor->visited = true;
        AddToOpen(neighbor);
    } 
}


/**
 * Sort the two-dimensional vector of ints in descending order.
 */
// Note: easier using lambda functios. need to understand this better
void RoutePlanner::listSort() {
  std::sort(open_list.begin(), open_list.end(), [](const auto &node1, const auto &node2){
      return node1->h_value + node1->g_value > node2->h_value + node2->g_value;
  });
}

// Complete the NextNode method to sort the open list and return the next node.

RouteModel::Node *RoutePlanner::NextNode() {
    //Sort the open_list according to the sum of the h value and g value.
    listSort();

    //Create a pointer to the node in the list with the lowest sum.
    auto *lowest_node = open_list.back();
    

    //Remove that node from the open_list.
    open_list.pop_back();
    
    //Return the pointer.
    return lowest_node;
}


// Complete the ConstructFinalPath method to return the final path found from your A* search.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    //This method should take the current (final) node as an argument and iteratively follow the 
    //chain of parents of nodes until the starting node is found.
    while(current_node->parent != nullptr){
        
        // add current node to the path
        //path_found.push_back(*current_node);
        path_found.insert(path_found.begin(),*current_node);

        //For each node in the chain, add the distance from the node to its parent to the distance variable.
        distance += current_node->distance(*current_node->parent);
        
        // Go to parent node (update current node)
        current_node = current_node->parent;
    }

    // add the last parent node
    //path_found.push_back(*current_node);
    //std::reverse(path_found.begin(), path_found.end());
    path_found.insert(path_found.begin(),*current_node);
 
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// Write the A* Search algorithm here.

void RoutePlanner::AStarSearch() {
    
    AddToOpen(start_node);
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    while(open_list.size() > 0){
        // Use the NextNode() method to sort the open_list and return the next node.
        current_node = NextNode();

        // Check if we're done. When the search has reached the end_node,
        if (current_node->distance(*end_node) == 0) {
            // use the ConstructFinalPath method to return the final path that was found.
            //Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
    
    // If we're not done, expand search to current node's neighbors. Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
    AddNeighbors(current_node);
  }  
  // We've run out of new nodes to explore and haven't found a path.
  std::cout << "No path found!" << "\n";
  return;
}