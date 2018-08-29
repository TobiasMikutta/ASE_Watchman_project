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
// ROS
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// Helper
#include <omp.h>
#include <boost/date_time/posix_time/posix_time.hpp>

#define _USE_MATH_DEFINES
#include <math.h>
//-------------------------------------------------------------------------------------------------------
//
//												Member-Klasse
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
class Beacon_Coord
{
	private:
		float x,y,winkel;
	public:
		 //Standard_Konstruktor
		 Beacon_Coord(){};
		 //Konstruktor
		 Beacon_Coord(float _x, float _y, float _winkel)
		 {
			set_values ( _x, _y,  _winkel);
		 };
		 //Setter
		 void set_values (float _x,float _y, float _winkel)
		 {
			x  = _x;
			y  = _y;
			winkel = _winkel;
		 }
		 //Getter
		 float get_x(void){return x;};
		 float get_y(void){return y;};
		 float get_winkel(void){return winkel;};
};
//-------------------------------------------------------------------------------------------------------
//
//											Beacon Listen Klasse	
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
class Beacon_List
{
	private:
		int id;
		static const int maxLimit = 9;
		Beacon_Coord Beacon[maxLimit];
	public:
		//Standard_Konstruktor
		Beacon_List(){};
		//Konstruktor
		Beacon_List(float _x, float _y, float _winkel, int _id)
		{
			set_Beacon( _x, _y,  _winkel,  _id);
		};
		//Setter
		void set_Beacon(float _x,float _y, float _winkel, int _id)
		{
			Beacon[_id].set_values( _x, _y, _winkel);
		}
		//Getter
		float get_winkel2(int _id){ Beacon[_id].get_winkel();};
		float get_x2(int _id){ Beacon[_id].get_x();};
		float get_y2(int _id){ Beacon[_id].get_y();};
};



using namespace std;
bool turn = false;
bool rechts = false;
bool links = false;
bool turned = false;


Beacon_List Beacon_Liste;


geometry_msgs::Twist  msg;

ros::Publisher pub;
ros::Publisher chatter_pub;
const size_t numSensors = 4;
std::random_device rd;
std::mt19937 gen(rd());
int idTest[2];


// Ros Listener Topic and publisher
string rosListenerTopicScan;
string rosListenerTopicOdom;
string rosPublisherTopic;
// Map variables
static int dimensionX = 9;    // (m) 
static int dimensionY = 9;    // (m)
static double mapResolution = 0.1; // (m)
static int mapDimensionX; // (cells)
static int mapDimensionY; // (cells)
static int mapEvaluationIter; // (1)
#define mattype uint16_t // CV_8U == uchar, CV_16U == short, CV_32FC1 == float
#define mattypecv CV_16U
#define maxValue 65535

static nav_msgs::OccupancyGrid ogm; // (P)

// Robot and sensor setup
static double robotOffsetX = dimensionX / 2.0f; // (m)
static double robotOffsetY = dimensionY / 2.0f; // (m)
static double maxReadingRange = 2.0f; // (m)
static double alpha = mapResolution; // (m/cell)
static double beta = 1 / 180.0 * M_PI; // (rad)

// program name
const string programName = "mapping_with_known_poses";

   int dec_inc 	= -1;
   int y_end 	= 0;
   double xx;
   double xy;
   double xt;
   double roll, pitch, yaw;
   int vorzeichen = 1;
   

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


enum Beacons
{
	Null 	= 0,
	Eins 	= 1,
	Zwei 	= 2,
	Drei 	= 3,
	Vier 	= 4,
	Fuenf 	= 5,
	Sechs 	= 6,
	Sieben 	= 7,
	Acht 	= 8,
	No_Point= 4294967295,
	Rand 	= 10
};

Beacons Punkt;






//-------------------------------------------------------------------------------------------------------
//
//								
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int classificatePoint(int leftSensorValue,int rightSensorValue, int midleft, int midright) //Berechnet die einzelnen WegPunkt ID's
{
    int class_value = -1;
	Punkt = Beacons(class_value);
	
	if((midleft < 30) && (midright < 30))
	{
		if(rightSensorValue>90&&rightSensorValue<110){class_value=0;}
		else if(rightSensorValue>115&&rightSensorValue<135){class_value=1;}
		else if(rightSensorValue>140&&rightSensorValue<160){class_value=2;}
		else
		{
			//return -1;
		}
		if(leftSensorValue>115&&leftSensorValue<135){class_value+=3;}
		else if(leftSensorValue>140&&leftSensorValue<160){class_value+=6;}
		else if(leftSensorValue>90&&leftSensorValue<110){class_value+=0;}
		else
		{
			//return -1;
		}
		
		Punkt = Beacons(class_value);
	}

	
	
	ROS_INFO("FAHREN");
	ros::NodeHandle n("~");
    //Publisher
	chatter_pub = n.advertise<geometry_msgs::Twist>("/amiro1/cmd_vel", 1);
	
    if (Punkt == 4294967295){
		//geometry_msgs::Twist  msg;
		//msg.linear.x = 0.1;
		//msg.angular.z=0.0;
		 //chatter_pub.publish(msg);
    }else{
		//geometry_msgs::Twist  msg;
		//msg.linear.x = 0.0;
		//msg.angular.z=0.0;
		//chatter_pub.publish(msg);
		}
	
    return class_value;
}
//-------------------------------------------------------------------------------------------------------
//
//								
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ExploreCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
   tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
   tf::Matrix3x3 m(q);
   m.getRPY(roll, pitch, yaw);
   double Drehung = 180/M_PI*yaw;
   if (Drehung < 0){Drehung = Drehung *-1;}
   
   
   
   ROS_INFO("Odometry");
     xx = odom->pose.pose.position.x;
     xy = odom->pose.pose.position.y;
     xt = odom->pose.pose.orientation.z;
  									
	ROS_INFO("Position x: [%f]", xx);
	ROS_INFO("Position y: [%f]", xy);
	ROS_INFO("Position z: [%f]", xt);

	ros::NodeHandle n("~");
	chatter_pub 	= n.advertise<geometry_msgs::Twist>("/amiro1/cmd_vel", 1);
	
	
	if((Punkt >=0) && (Punkt <=8) )
    {
		Beacon_Liste.set_Beacon(xx,xy,Drehung,Punkt);
		
		float winkel = Beacon_Liste.get_winkel2(Punkt);
		float x = Beacon_Liste.get_x2(Punkt);
		float y = Beacon_Liste.get_y2(Punkt);
		
		ROS_INFO("ID: %d",Punkt); 
		ROS_INFO("WINKEL: %f",winkel); 
		ROS_INFO("X: %f",x); 
		ROS_INFO("Y: %f",y); 
	}
	else
	{
		float winkel = Beacon_Liste.get_winkel2(0);
		float x = Beacon_Liste.get_x2(0);
		float y = Beacon_Liste.get_y2(0);
		
		
		ROS_INFO("X: %f",x); 
		ROS_INFO("Y: %f",y); 
		
		 winkel = Beacon_Liste.get_winkel2(1);
		 x = Beacon_Liste.get_x2(1);
		 y = Beacon_Liste.get_y2(1);
		
		ROS_INFO("X: %f",x); 
		ROS_INFO("Y: %f",y); 
		
		 winkel = Beacon_Liste.get_winkel2(2);
		 x = Beacon_Liste.get_x2(2);
		 y = Beacon_Liste.get_y2(2);
		 
		ROS_INFO("X: %f",x); 
		ROS_INFO("Y: %f",y); 
		
		winkel = Beacon_Liste.get_winkel2(3);
		 x = Beacon_Liste.get_x2(3);
		 y = Beacon_Liste.get_y2(3);
		

		ROS_INFO("X: %f",x); 
		ROS_INFO("Y: %f",y); 
		
		winkel = Beacon_Liste.get_winkel2(4);
		 x = Beacon_Liste.get_x2(4);
		 y = Beacon_Liste.get_y2(4);
		
		ROS_INFO("X: %f",x); 
		ROS_INFO("Y: %f",y); 
		
		winkel = Beacon_Liste.get_winkel2(5);
		 x = Beacon_Liste.get_x2(5);
		 y = Beacon_Liste.get_y2(5);

		ROS_INFO("X: %f",x); 
		ROS_INFO("Y: %f",y); 
		
		winkel = Beacon_Liste.get_winkel2(6);
		 x = Beacon_Liste.get_x2(6);
		 y = Beacon_Liste.get_y2(6);
		

		ROS_INFO("X: %f",x); 
		ROS_INFO("Y: %f",y); 
		
		winkel = Beacon_Liste.get_winkel2(7);
		 x = Beacon_Liste.get_x2(7);
		 y = Beacon_Liste.get_y2(7);
		

		ROS_INFO("X: %f",x); 
		ROS_INFO("Y: %f",y); 
	}
	
	/*
	//Fährt hin und her von Wand zur Wand	 (links nach rechts perfekt, rechts nach links auf Droge)
	if ((xx < 5) && (xx > 0) && (turned == false) && (turn == false) )
	{
		msg.angular.z= vorzeichen * yaw; 
		msg.linear.x = 0.1;	
	}
	//Wenn außerhalb des Bereichs.. 
	if(!((xx <= 5) && (xx >= 0)) && (turned == false))
	{
		turn = true;
		//rechts dann dreh dich links
		if(xx >= 5)
		{
			links = true;
			rechts = false;
			msg.linear.x 	= 0.0;
			msg.angular.z	= 1.0;	
		}
		//andernfalls dreh dich rechts
		else
		{
			links = false;
			rechts = true;
			msg.linear.x 	= 0.0;
			msg.angular.z	= -1.0;	
		}
	}
	//wenn du dich rechts drehen sollst und
	if ((turn == true) && (rechts == true))
	{
		//der Winkel nun negativ wird hör auf 
		if (Drehung < 10)
		{
			msg.angular.z	= 0.0;	
			msg.linear.x 	= 0.1;
			turn = false;
			rechts = false;
			turned = true;
		}
	}
	//wenn du dich links herum drehen sollst und
	else if ((turn == true) && (links == true))
	{
		//der Winkel nun positiv wird hör auf
		if (Drehung > 170)
		{
			msg.angular.z	= 0.0;	
			msg.linear.x 	= 0.1;
			turn = false;
			links = false;
			turned = true;
		}
	}
	//wenn gewendet und im Bereich dann normalbetrieb
	if ((turned == true) && (xx > 0) && (xx < 5))
	{
		turned = false;
		//vorzeichen = vorzeichen * -1;
	}*/

	chatter_pub.publish(msg);
return;
}
//-------------------------------------------------------------------------------------------------------
//
//								
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
  

	int id=classificatePoint(idTest[0],idTest[1],values.array.data[0],values.array.data[3] );
  
    if(id!=-1)
	{
		ROS_INFO("Point ID: %u",id);
		ROS_INFO("Beacon ID: %u",Punkt);
	}else
	{
		ROS_INFO("Point ID: NO POINT");
		ROS_INFO("Beacon ID: %u",Punkt);
	}
 
  pub.publish(values);
  return;
}
//-------------------------------------------------------------------------------------------------------
//
//								
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int main(int argc, char **argv)
{
  ROS_INFO("MAIN");
	
  ros::init(argc, argv, "proximity_ring_mockup_node");
  ros::NodeHandle n("~");
  ros::NodeHandle node("~");
  ros::NodeHandle n2("~");
  ros::NodeHandle n3("~");
  
  ROS_INFO("Started proximity_ring_mockup_node");
  
  ros::Subscriber sub = n2.subscribe("/amiro1/odom", 1000, ExploreCallback);
  



  
  
  
  
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

  // Subscriber:  floor topic
  message_filters::Synchronizer<syncPolicy> sync(syncPolicy(1000), sub0, sub1, sub2, sub3);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  pub = n.advertise<amiro_msgs::UInt16MultiArrayStamped>(topic_out, 1);
	
  ros::spin();

  return 0;
}
