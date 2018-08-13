#include "ros/ros.h"


struct Point
{
    int x, y;
    int id;
    Point(int x_, int y_) : x(x_), y(y_) {}
}
//------------------------------------------------------------------------



//------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "amiro1_move_to_point");

  int x_max, y_max;

}
//------------------------------------------------------------------------



//------------------------------------------------------------------------
int classificatePoint(int leftSensorValur,int rightSensorValue) //Berechnet die einzelnen WegPunkt ID's
{
    int class_value = 0;

    class_value = (rightSensorValue-100)/25;

    switch (leftSensorValur)
    {
        case 125: class_value =+ 3;
                break;
        case 150: class_value =+ 6;
                break;
        default:  
                break;
    } 

    return class_value;
}
//------------------------------------------------------------------------



//------------------------------------------------------------------------
Point[] explorateMap()
{
    int x, y;

    int iterator = 10;

    determine_Map_Size(); //find_corner (find wall -> wall following -> set p oint zero)

    for(x = 0; Wall_not_detected;  x =+ 10)
    {
        for(y = 0; Wall_not_detected; y =+ iterator)
        {
            
        }
        iterator =* -1;
    }

    return Points;
}
//------------------------------------------------------------------------



//------------------------------------------------------------------------
int[] determine_Map_Size()
{
    find_Corner();

    set_Point_Zero();

    Wall_Following();

    set_Map_Size();
}

