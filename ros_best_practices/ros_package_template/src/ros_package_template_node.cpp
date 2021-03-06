/*#include "ros/ros.h"
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

static uint floorSensor[4];
static uint normFloorSensor[4];
int maxFloorValue=46000;

uint32_t Buffer1[25];
uint32_t Mittelwert;

int counter = 0;

inline double gray2mean(const double gray) {
  const double a = 11010.7641914599;
  const double b = 0.0610333423539444;
  const double c = 21783.9217626851;
  const double d = -7.94411704377273;
  return a*atan(b*gray+d)+c;
}

inline double gray2std(const double gray) {
  const double a = 1108.83338439758;
  const double b = 0.0223713977792142;
  const double c = 1503.24485506827;
  const double d = -2.08504831316051;
  return a*tanh(b*gray+d)+c;
}


void floorSubCallback(const amiro_msgs::UInt16MultiArrayStamped::ConstPtr floor) {  
	
  
      Buffer1[counter] = floor->array.data[1];
      counter++;
  
	if (counter == 10)
	{
		counter = 0;
		
		for(int o = 0; o < 10; o++)
		{
			Mittelwert += Buffer1[o];
		}
		
		Mittelwert /= 10;
		Mittelwert=( double (Mittelwert) /double( maxFloorValue))*255.0;
		
		ROS_INFO("Bodensensor: %u.",Mittelwert);
		
		
		
		/*for (uint i = 0; i < floor->array.data.size(); i++)
		{
		  floorSensor[i] = floor->array.data[i];
		  
		  if(floorSensor[i]>maxFloorValue)
		  {
			maxFloorValue=floorSensor[i];
		  }
		  
		   floorSensor[i] =(double (floorSensor[i])/double( maxFloorValue))*255.0;
		   ROS_INFO("Bodensensor[%u]: %d.", i,floorSensor[i]);
		   ROS_INFO("Bodensensor MAX[%u]",maxFloorValue); 
		
	}
		
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 

int main(int argc, char **argv)
{
	int amiroId;
	  
    ros::init(argc,argv,"talker");

    ros::NodeHandle n;
    
    ros::NodeHandle node("~");
    
    //Publisher
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/amiro1/cmd_vel", 1);
    
    geometry_msgs::Twist  msg;
    
    //msg.linear.x = 0.1;
    
    chatter_pub.publish(msg);
    
    //Subscriber  
	ros::Subscriber floor_sub = node.subscribe("/amiro1/proximity_floor/values", 1, floorSubCallback);
  


	ros::spin();
	
  return 0;
}



/*
void callback(const sensor_msgs::Image::ConstPtr msg0,
              const sensor_msgs::Image::ConstPtr msg1,
              const sensor_msgs::Image::ConstPtr msg2,
              const sensor_msgs::Image::ConstPtr msg3) {

  const sensor_msgs::Image::ConstPtr msgs[numSensors] = {msg0, msg1, msg2, msg3};

  amiro_msgs::UInt16MultiArrayStamped values;
  values.array.data.resize(size(msgs));
  values.header = msgs[0]->header;
  values.header.frame_id = "";

  for (std::size_t idx = 0; idx < numSensors; ++idx) {

    // Integrate over all pixel values to get the mean gray value
    size_t grayIntegrated = 0;
    for(auto it = msgs[idx]->data.begin(); it != msgs[idx]->data.end(); ++it) {
      grayIntegrated += size_t(*it);
    }
    // Normalize to 0 .. 255
    const double gray = double(grayIntegrated) / msgs[idx]->data.size();

    // Parameterize the normal distribution to sample
    std::normal_distribution<> distribution(gray2mean(gray),gray2std(gray));

    // Sample the sensor value
    const double sensorValue = distribution(gen);

    // Truncate the sampled value
    if (sensorValue < double(std::numeric_limits<unsigned short>::min())) {
      values.array.data.at(idx) = std::numeric_limits<unsigned short>::min();
    } else if (sensorValue > double(std::numeric_limits<unsigned short>::max())) {
      values.array.data.at(idx) = std::numeric_limits<unsigned short>::max();
    } else {
      values.array.data.at(idx) = std::round(sensorValue);
    }

    // Get the most current timestamp for the whole message
    if (values.header.stamp < msgs[idx]->header.stamp) {
      values.header.stamp = msgs[idx]->header.stamp;
    }
  }
  pub.publish(values);
}*/
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

ros::Publisher pub;
const size_t numSensors = 4;
std::random_device rd;
std::mt19937 gen(rd());
int idTest[2];
inline double gray2mean(const double gray) {
  const double a = 11010.7641914599;
  const double b = 0.0610333423539444;
  const double c = 21783.9217626851;
  const double d = -7.94411704377273;
  return a*atan(b*gray+d)+c;
}

inline double gray2std(const double gray) {
  const double a = 1108.83338439758;
  const double b = 0.0223713977792142;
  const double c = 1503.24485506827;
  const double d = -2.08504831316051;
  return a*tanh(b*gray+d)+c;
}

int classificatePoint(int leftSensorValue,int rightSensorValue) //Berechnet die einzelnen WegPunkt ID's
{
    int class_value = -1;

    if(rightSensorValue>90&&rightSensorValue<110){class_value=0;}
    else if(rightSensorValue>115&&rightSensorValue<135){class_value=1;}
    else if(rightSensorValue>140&&rightSensorValue<160){class_value=2;}
    else
    {
		return -1;
	}
    if(leftSensorValue>115&&leftSensorValue<135){class_value+=3;}
    else if(leftSensorValue>140&&leftSensorValue<160){class_value+=6;}
    else if(leftSensorValue>90&&leftSensorValue<110){class_value+=0;}
	else
    {
		return -1;
	}
	
	
    return class_value;
}

void callback(const sensor_msgs::Image::ConstPtr msg0,
              const sensor_msgs::Image::ConstPtr msg1,
              const sensor_msgs::Image::ConstPtr msg2,
              const sensor_msgs::Image::ConstPtr msg3) {

  const sensor_msgs::Image::ConstPtr msgs[numSensors] = {msg0, msg1, msg2, msg3};

  amiro_msgs::UInt16MultiArrayStamped values;
  values.array.data.resize(size(msgs));
  values.header = msgs[0]->header;
  values.header.frame_id = "";

  for (std::size_t idx = 0; idx < numSensors; ++idx) {

    // Integrate over all pixel values to get the mean gray value
    size_t grayIntegrated = 0;
    for(auto it = msgs[idx]->data.begin(); it != msgs[idx]->data.end(); ++it) {
      grayIntegrated += size_t(*it);
    }
    // Normalize to 0 .. 255
    const double gray = double(grayIntegrated) / msgs[idx]->data.size();
	
	 values.array.data.at(idx) = uint(gray);

  }
  idTest[0]= values.array.data[1];
  idTest[1]= values.array.data[2];
  int id=classificatePoint(idTest[0],idTest[1]);
  if(id!=-1)
	{
		ROS_INFO("Point ID: %u",id);
	}else
	{
		ROS_INFO("Point ID: NO POINT");
	}
  pub.publish(values);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "proximity_ring_mockup_node");
  ros::NodeHandle n("~");

  ROS_INFO("Started proximity_ring_mockup_node");

  std::string topic_out, topic_in_suffix, topic_in_prefix;

  n.param<std::string>("topic_in_suffix", topic_in_suffix, "/amiro1/proximity_floor_");
  n.param<std::string>("topic_in_prefix", topic_in_prefix, "/image_raw");
  n.param<std::string>("topic_out", topic_out, "/amiro666/proximity_floor/values");

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,
      sensor_msgs::Image,
      sensor_msgs::Image,
      sensor_msgs::Image> syncPolicy;

  message_filters::Subscriber<sensor_msgs::Image> sub0(n, topic_in_suffix + "0" + topic_in_prefix, 1);
  message_filters::Subscriber<sensor_msgs::Image> sub1(n, topic_in_suffix + "1" + topic_in_prefix, 1);
  message_filters::Subscriber<sensor_msgs::Image> sub2(n, topic_in_suffix + "2" + topic_in_prefix, 1);
  message_filters::Subscriber<sensor_msgs::Image> sub3(n, topic_in_suffix + "3" + topic_in_prefix, 1);

  message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub0, sub1, sub2, sub3);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  pub = n.advertise<amiro_msgs::UInt16MultiArrayStamped>(topic_out, 1);
   
   

    //Publisher
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/amiro1/cmd_vel", 1);
    
    geometry_msgs::Twist  msg;
    
    msg.linear.x = 0.0001;
    
    chatter_pub.publish(msg);
  ros::spin();

  return 0;
}
