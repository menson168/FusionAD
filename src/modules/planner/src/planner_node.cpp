#include "planner/planner_node.h"

int main(int argc, char **argv)
{
  //ROS node initialization as "control core"
  ros::init(argc, argv, "core_planner");
  /*
  fusionad::control::node::ControlNode core_control;
  if(!core_control.getParameter())
  {
    ROS_FATAL("Parameter Failed! Abort!");
    core_control.externalFailFlag = true;
  }
  core_control.initRosComm();
  */
  while(ros::ok())
  {
    //Execute all callbacks first to initiate the variables
    ros::spinOnce();
  }
  return 0;
}