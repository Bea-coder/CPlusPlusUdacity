#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
  RouteModel::Node *s_Node=&m_Model.FindClosestNode(start_x,start_y);
  RouteModel::Node *e_Node=&m_Model.FindClosestNode(end_x,end_y);
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
  this->start_node=s_Node;
  this->end_node=e_Node;
  std::cout<<"Start node :"<<start_node->x<<","<<start_node->y<<"\n";
  std::cout<<"End node   :"<<end_node->x<<","<<end_node->y<<"\n";


}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  float distance=node->distance(*end_node);
  return distance;
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node* current_node) {
	current_node->FindNeighbors();
    for(auto &node:current_node->neighbors){
      if(node->visited!=true){
        node->parent=current_node;
        node->h_value=this->CalculateHValue(node);
        node->g_value=current_node->g_value+node->distance(*current_node);
        node->visited=true;
        this->open_list.push_back(node);
      }
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.
bool Compare(RouteModel::Node* current , RouteModel::Node* other){ 
  return (current->g_value+current->h_value)>(other->g_value+other->h_value);
}



RouteModel::Node* RoutePlanner::NextNode() {
 
  std::sort(this->open_list.begin(),this->open_list.end(),Compare);
  int size=open_list.size();
  RouteModel::Node* lowest_sum=this->open_list[size-1];
  this->open_list.pop_back();
  return lowest_sum;
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
    RouteModel::Node* node=current_node;
    RouteModel::Node* parent=current_node->parent;
    while(node != nullptr){
      	parent=node->parent;
        path_found.insert(path_found.begin(),*node);
      	if(node->distance(*start_node) >0.01){
		  distance+=node->distance(*parent);
          node=parent;
        }else {
   //       std::cout<<"End\n";
          node=nullptr;
          break;
        }    	
    }
 // 	std::cout<<"Out loop\n";
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    //std::cout<<"Final distance "<<distance<<"\n";
  	return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node* current_node = nullptr;
    // TODO: Implement your solution here.
  	//Initialize starting node
  	current_node=start_node;
  	current_node->parent=nullptr;
  	current_node->g_value=0;
  	current_node->h_value=this->CalculateHValue(current_node);
    current_node->visited=true;
    open_list.push_back(current_node);
  	while(this->open_list.size()>0){
    	current_node=NextNode();
  	  	if(current_node->distance(*end_node)==0){
        //  std::cout<<"------>path found\n";
      	  this->m_Model.path=ConstructFinalPath(current_node);
          return;
        }else{
          this->AddNeighbors(current_node);
        }  
    }
}
