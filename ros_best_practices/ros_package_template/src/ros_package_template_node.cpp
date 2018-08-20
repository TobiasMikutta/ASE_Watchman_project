#include "ros/ros.h"
#include <limits>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <random>
#include <cmath>
#include "sensor_msgs/Image.h"
#include "std_msgs/UInt16.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <amiro_msgs/UInt16MultiArrayStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Twist.h>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

void chatterCallback(const amiro_msgs::UInt16MultiArryStamped::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/amiro1/proximity_floor/values", 1000, chatterCallback);

  return 0;
}
