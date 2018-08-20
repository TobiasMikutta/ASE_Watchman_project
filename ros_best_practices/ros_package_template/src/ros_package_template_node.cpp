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

int main(int argc, char **argv)
{
    ros::init(argc,argv,"talker");

    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/amiro1/cmd_vel", 1);
    geometry_msgs::Twist  msg;
    msg.linear.x = 0.1;
    while(ros::ok()){

    chatter_pub.publish(msg);
 }

  return 0;
}
