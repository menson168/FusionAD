#include "planner/planner_lib.h"

using namespace fusionad::planner::libraries;

Graph::Node::Node(){}

Graph::Node::Node(std::string name)
{
  NodeName = name;  
}

Graph::Node::Node(std::string name, std::pair<float, float> xy_position)
{
  NodeName = name;  
  cartesianPosition = xy_position;
}

Graph::Node::~Node(){}


Graph::Graph(){}

Graph::Graph(std::vector<Graph::Node*> listOfNodePtr)
{
  NodeList = listOfNodePtr;
}

Graph::~Graph()
{
}

bool Graph::addNode(Node *Node_ptr)
{
  // Check for any duplicate in the current node list
  std::vector<Node*>::iterator find_itr = std::find(NodeList.begin(), NodeList.end(), Node_ptr);

  if(find_itr == NodeList.end())
  {
    // No duplicate is found if the find function returns end iterator of the node list
    NodeList.push_back(Node_ptr);
    return true;
  }
  else
  {
    return false;
  }
}

bool Graph::removeNode(Node *Node_ptr)
{
  // Check the node intended to be removed exists in current list
  std::vector<Node*>::iterator find_itr = std::find(NodeList.begin(), NodeList.end(), Node_ptr);  
  if(find_itr == NodeList.end())
  {
    // Node desired to be removed does not exist in the current node list
    return false;
  }
  else
  {
    NodeList.erase(find_itr);
    return true;
  }  
}

bool Graph::addConnection(Node *Node_ptr_1, Node *Node_ptr_2, float& weight)
{
  // Check if node exists first
  if((std::find(NodeList.begin(), NodeList.end(), Node_ptr_1) != NodeList.end()) 
    && (std::find(NodeList.begin(), NodeList.end(), Node_ptr_2) != NodeList.end()))
  {
    // Check if either node exists in another node's connection list already
    if((Node_ptr_1->connections.find(Node_ptr_2) != Node_ptr_1->connections.end()) 
      && (Node_ptr_2->connections.find(Node_ptr_1) != Node_ptr_2->connections.end()))
    {
      // Add node to its respective list
      Node_ptr_1->connections.emplace(Node_ptr_2, weight);
      Node_ptr_2->connections.emplace(Node_ptr_1, weight);
      return true;
    }
  }
  return false;
}

bool Graph::removeConnection(Node *Node_ptr_1, Node *Node_ptr_2)
{
  // Add node to its respective list
  return (Node_ptr_1->connections.erase(Node_ptr_1) && Node_ptr_2->connections.erase(Node_ptr_2));
}

std::pair<std::vector<Graph::Node*>::const_iterator,std::vector<Graph::Node*>::const_iterator> Graph::getNodeListBeginAndEndIterator()const
{
  if(NodeList.begin() == NodeList.end())
  {
    throw std::logic_error("Node list Is Empty!");
  }
  else
  { 
    return std::make_pair(NodeList.begin(),NodeList.end());
  } 
}

bool Graph::isNodeListEmpty()const
{
  return NodeList.empty();
}

AStarPlanner::AStarPlanner()
{
}

AStarPlanner::~AStarPlanner()
{
}

bool AStarPlanner::setGraph(Graph* Graph_ptr)
{
  targetSearchSpace_ptr = Graph_ptr;
  return true;
}

std::vector<Graph::Node*> AStarPlanner::searchForShortestPath(Grpah::Node* currentNode_ptr, Graph::Node* targetNode_ptr)
{
  if(!targetSearchSpace_ptr->isNodeListEmpty())
  {
    // Only do search if the current node list in the graph is not empty
    
  }
  else
  {
    throw std::logic_error("Node list Is Empty!");
  }
} 