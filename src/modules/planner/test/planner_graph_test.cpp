#include <gtest/gtest.h>
#include <planner/planner_lib.h>
#include <iostream>

using namespace std;
using namespace fusionad::planner::libraries;

// Create a graph 
Graph test_graph;

Graph::Node first_node("first_node");
Graph::Node second_node("second_node");
Graph::Node third_node("third_node");
Graph::Node fourth_node("fourth_node");
Graph::Node fifth_node("fifth_node");   

/**     
 * Graph
 * 
 * 3-"2"-4-"5"-1
 * |     |    /
 *"1"   "9" "7"
  * |     |  /
  * |     | /
  * 2-"3"-5
  * 
  */

// Create connection for each of the node
std::unordered_map<Graph::Node*, float> first_adj_list;
std::unordered_map<Graph::Node*, float> second_adj_list;
std::unordered_map<Graph::Node*, float> third_adj_list;
std::unordered_map<Graph::Node*, float> fourth_adj_list;
std::unordered_map<Graph::Node*, float> fifth_adj_list;


TEST(GraphTest_CreateNodeTestWithName, ShouldPass)
{
  // Test cases
  ASSERT_EQ(first_node.NodeName,"first_node")<<"First Node Name: "<<first_node.NodeName;
  ASSERT_EQ(second_node.NodeName,"second_node")<<"Second Node Name: "<<second_node.NodeName;
  ASSERT_EQ(third_node.NodeName,"third_node")<<"Third Node Name: "<<third_node.NodeName;
  ASSERT_EQ(fourth_node.NodeName,"fourth_node")<<"Fourth Node Name: "<<fourth_node.NodeName;
  ASSERT_EQ(fifth_node.NodeName,"fifth_node")<<"Fifth Node Name: "<<fifth_node.NodeName;
}

TEST(GraphTest_AddNodeConnection, ShouldPass)
{
  // first_adj_list
  first_adj_list.emplace(&fourth_node, 5);    // Solution to first node test
  first_adj_list.emplace(&fifth_node, 7);

  // second_adj_list
  second_adj_list.emplace(&third_node, 1);    // Solution to the second node test
  second_adj_list.emplace(&fifth_node, 3);

  //third_adj_list
  third_adj_list.emplace(&second_node, 1);    // Solution to the third node test
  third_adj_list.emplace(&fourth_node, 2);

  //fourth_adj_list
  fourth_adj_list.emplace(&fifth_node, 9);   // Solution to the fourth node test
  fourth_adj_list.emplace(&first_node, 5);    
  fourth_adj_list.emplace(&third_node, 2);    

  //fifth_adj_list
  fifth_adj_list.emplace(&fourth_node,9);     // Solution to the fifth node test
  fifth_adj_list.emplace(&second_node,3);
  fifth_adj_list.emplace(&first_node, 8);

  // Stuff these adj list into their own node
  first_node.connections = first_adj_list;
  second_node.connections = second_adj_list;
  third_node.connections = third_adj_list;
  fourth_node.connections = fourth_adj_list;
  fifth_node.connections = fifth_adj_list;

  // First node test
  auto first_find = first_node.connections.find(&fourth_node);
  ASSERT_EQ(first_find->first, &fourth_node);
  ASSERT_EQ(first_find->first->NodeName, "fourth_node");
  ASSERT_EQ(first_find->second, 5);

  // Second node test
  auto second_find = second_node.connections.find(&third_node);
  ASSERT_EQ(second_find->first, &third_node);
  ASSERT_EQ(second_find->first->NodeName, "third_node");  
  ASSERT_EQ(second_find->second, 1);

  // Third node test
  auto third_find = third_node.connections.find(&second_node);
  ASSERT_EQ(third_find->first, &second_node);
  ASSERT_EQ(third_find->first->NodeName, "second_node");    
  ASSERT_EQ(third_find->second, 1);

  // Fourth node test
  auto fourth_find = fourth_node.connections.find(&fifth_node);
  ASSERT_EQ(fourth_find->first, &fifth_node);
  ASSERT_EQ(fourth_find->first->NodeName, "fifth_node");      
  ASSERT_EQ(fourth_find->second, 9);

  // Fifth node test
  auto fifth_find = fifth_node.connections.find(&fourth_node);
  ASSERT_EQ(fifth_find->first, &fourth_node);
  ASSERT_EQ(fifth_find->first->NodeName, "fourth_node");    
  ASSERT_EQ(fifth_find->second, 9);
}


TEST(GraphTest_AddNodeTest, ShouldPass)
{
  // Add the node into the list
  test_graph.addNode(&first_node);
  test_graph.addNode(&second_node);
  test_graph.addNode(&third_node);
  test_graph.addNode(&fourth_node);
  test_graph.addNode(&fifth_node);

  std::pair<std::vector<Graph::Node*>::const_iterator,std::vector<Graph::Node*>::const_iterator> node_list_iterator_pair;
  node_list_iterator_pair = test_graph.getNodeListBeginAndEndIterator();

  ASSERT_EQ(*node_list_iterator_pair.first, &first_node);
  ASSERT_EQ(*node_list_iterator_pair.first+1, &second_node);
  ASSERT_EQ(*node_list_iterator_pair.first+2, &third_node);
  ASSERT_EQ(*node_list_iterator_pair.first+3, &fourth_node);
  ASSERT_EQ(*node_list_iterator_pair.first+4, &fifth_node);
} 

// Main function
int main(int argc, char **argv) 
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}