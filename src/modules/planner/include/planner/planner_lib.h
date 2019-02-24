#ifndef PLANNER_LIB_H
#define PLANNER_LIB_H

#include "nav_msgs/Path.h"
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <string>
#include <algorithm>
#include <utility>
#include <unordered_map>
#include <exception>

namespace fusionad
{
namespace planner
{
namespace libraries
{


class Graph
{
  public:
    class Node;
    Graph();
    Graph(std::vector<Graph::Node*> listOfNodePtr);
    ~Graph();
    bool addNode(Node *Node_ptr);
    bool removeNode(Node *Node_ptr);
    bool addConnection(Node *Node_ptr_1, Node *Node_ptr_2, float& weight);
    bool removeConnection(Node *Node_ptr_1, Node *Node_ptr_2);
    bool isNodeListEmpty()const;
    std::pair<std::vector<Graph::Node*>::const_iterator,std::vector<Graph::Node*>::const_iterator> getNodeListBeginAndEndIterator()const;
  private:
    // Create a node list of a vector that contains node pair ptrs
    std::vector<Graph::Node*> NodeList;
};

class Graph::Node
{
  public:
    Node();
    Node(std::string name);
    Node(std::string name, std::pair<float, float> xy_position);
    ~Node();
    std::string NodeName;
    std::unordered_map<Node*, float> connections;
    std::pair<float, float> cartesianPosition;    // First->X, Second->Y  
};

class AStarPlanner
{
  public:
    AStarPlanner();
    ~AStarPlanner();
    bool setGraph(Graph* Graph_ptr);
    std::vector<Graph::Node*> searchForShortestPath(Grpah::Node* currentNode_ptr, Graph::Node* targetNode_ptr);

  private:
    Graph* targetSearchSpace_ptr;

};

class VirtualRail
{
  public:
    VirtualRail();
    ~VirtualRail();
  private:
    nav_msgs::Path convertNodeListToPath();
};

}
}
}

#endif