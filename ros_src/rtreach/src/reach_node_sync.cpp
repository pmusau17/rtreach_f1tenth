#include "ros/ros.h"
#include <iostream>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <rtreach/angle_msg.h>
#include <rtreach/velocity_msg.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// The following class will receive messages from the LEC which will be prediction of the steering angle
// It will also receive messages from the speed node which will dictate how fast the car should travel
// Initially the assmumption will be the the car moves at constant velocity

extern "C"
{
     double getSimulatedSafeTime(double start[4],double heading_input,double throttle);
     bool runReachability_bicycle(double * start, double  simTime, double  wallTimeMs, double  startMs,double  heading_input, double  throttle);
}




void callback(const nav_msgs::Odometry::ConstPtr& msg, const rtreach::velocity_msg::ConstPtr& velocity_msg, const rtreach::angle_msg::ConstPtr&)
{
  using std::cout;
  using std::endl;

  cout << "received both messages" << endl;
}


int main(int argc, char **argv)
{

    using namespace message_filters;

    // initialize the ros node
    ros::init(argc, argv, "reachnode_sync");



    /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */

    ros::NodeHandle n;



    // a description of how the synchronization works can be found here: 
    // http://wiki.ros.org/message_filters/ApproximateTime
    
    
    // define the subscribers you want 
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(n, "racecar/odom", 10);
    message_filters::Subscriber<rtreach::velocity_msg> vel_sub(n, "racecar/velocity_msg", 10);
    message_filters::Subscriber<rtreach::angle_msg> angle_sub(n, "racecar/angle_msg", 10);


    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, rtreach::velocity_msg, rtreach::angle_msg> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, vel_sub,angle_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2,_3));


    ros::spin();


    return 0; 
}