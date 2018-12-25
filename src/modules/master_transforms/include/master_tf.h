#ifndef MASTER_TF_H
#define MASTER_TF_H

/*
NOTE: This node prepares the tf message to broadcast by using the input of the rotated yaw message
      (according to REP-105) and performing the 2D rotation required from sensor location to front-axle. 
      Only the x-position is offset (with respect to the vehicle on the red car) so there is no y-position component in the 
      2-D transform. 

INPUTS: TOPIC:  /localization/rotated_yaw
                Msg: std_msgs::Float32
        TOPIC:  /localization/calibrated_yaw
                Msg: std_msgs::Float32

OUTPUTS: Geodesy TF Broadcast
         Lidar TF Broadcast
*/

#include "ros/ros.h"
#include "math.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

namespace fusionad
{
namespace master_tf
{
namespace master_tf_node
{
class MasterTfNode
{
    public:
        MasterTfNode();
        ~MasterTfNode();
        void initRosComm();
    
    private:
        // initializing the nodehandle
        ros::NodeHandle masterTfNode_nh;

        // initialize the timer
        ros::Timer master_tf_timer;

        // initialize the subscribers
        ros::Subscriber calibrated_yaw_sub;
        ros::Subscriber rotated_yaw_sub;
        
        tf::TransformBroadcaster geodesy_broadcaster;
        tf::TransformBroadcaster lidar_broadcaster;

        float calibrated_x_pose;
        float calibrated_y_pose;
        float calibrated_yaw;
        float rot_yaw;
        bool calibration_complete = false;

        // initialize callback functions for calibrated messages from the frame_calibration node
        void yawCallback(const std_msgs::Float32& cal_yaw_msg);
        void rotatedYawCallback(const std_msgs::Float32& rot_yaw_msg);
        void timerCallback(const ros::TimerEvent& event);

}; // MasterTfNode
} // master_tf_node
} // master_tf
} // fusionad

#endif
