#include "ros/ros.h"
#include <iostream>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

// This will be my introduction to subscribing to messages using C++
// The following class will receive messages from the LEC which will be prediction of the steering angle
// It will also receive messages from the speed node which will dictate how fast the car should travel
// Initially the assmumption will be the the car moves at constant velocity


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{  
  using std::cout;
  using std::endl;

  cout << "x: " << msg-> pose.pose.position.x << endl;
  cout << "y: " << msg-> pose.pose.position.y << endl;
}

// * The ros::init() function needs to see argc and argv so that it can use
// * any ROS arguments and name remapping that were provided at the command line.
 
int main(int argc, char **argv)
{

    // initialize the ros node
    ros::init(argc, argv, "reachnode");



    /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */

    ros::NodeHandle n;


    // initialize the subscriber 
    ros::Subscriber sub = n.subscribe("racecar/odom", 1000, odomCallback);

    ros::spin();


    return 0; 
}





