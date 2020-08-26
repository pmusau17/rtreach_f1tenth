#include "ros/ros.h"
#include <iostream>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

// This will be my introduction to subscribing to messages using C++
// The following class will receive messages from the LEC which will be prediction of the steering angle
// It will also receive messages from the speed node which will dictate how fast the car should travel
// Initially the assmumption will be the the car moves at constant velocity


extern "C"
{
     double getSimulatedSafeTime(double start[4],double heading_input,double throttle);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{  
  using std::cout;
  using std::endl;
  double roll, pitch, yaw, lin_speed;
  double x,y,u,delta;

  x = msg-> pose.pose.position.x;
  y = msg-> pose.pose.position.y;

  cout << "x: " << x << endl;
  cout << "y: " << y << endl;

  // define the quaternion matrix
  tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);

 
  // convert to rpy
  m.getRPY(roll, pitch, yaw);

  cout << "yaw: " << yaw << endl;

  // normalize the speed 
  tf::Vector3 speed = tf::Vector3(msg->twist.twist.linear.x, msg->twist.twist.linear.x, 0.0);
  lin_speed = speed.length();

  cout << "speed: " << lin_speed << endl;


  // initialize the start state

  double state[4] = {x,y,lin_speed,yaw};
  //double control[2] = {}

  getSimulatedSafeTime(state,0.2666,16.0);
}

void ackermannCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
{
    using std::cout;
    using std::endl;

    cout << "throttle: " << msg->drive.speed << endl;
    cout << "steering angle: " << msg->drive.steering_angle << endl;
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
    ros::Subscriber sub2 = n.subscribe("vesc/ackermann_cmd_mux/input/teleop",1000,ackermannCallback);

    ros::spin();


    return 0; 
}





