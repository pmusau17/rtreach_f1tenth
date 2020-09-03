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
#include <ros/package.h>


// The following node will receive messages from the LEC which will be prediction of the steering angle
// It will also receive messages from the speed node which will dictate how fast the car should travel
// Initially the assmumption will be the the car moves at constant velocity

extern "C"
{ 
     #include "bicycle_safety.h"
     double getSimulatedSafeTime(double start[4],double heading_input,double throttle);
     bool runReachability_bicycle(double * start, double  simTime, double  wallTimeMs, double  startMs,double  heading_input, double  throttle);
     void deallocate_2darr(int rows,int columns);
     void load_wallpoints(const char * filename, bool print);
}

/**
* NodeHandle is the main access point to communications with the ROS system.
* The first NodeHandle constructed will fully initialize this node, and the last
* NodeHandle destructed will close down the node.
*/


ros::Publisher ackermann_pub; 

// reachability parameters
const double sim_time = 1.5;
double ms = 0.0; // this is redundant will remove in refactoring
const double walltime = 80; // this in ms apparently wtf the declaration doesn't say that 

bool stop = false;


void callback(const nav_msgs::Odometry::ConstPtr& msg, const rtreach::velocity_msg::ConstPtr& velocity_msg, const rtreach::angle_msg::ConstPtr& angle_msg)
{
  using std::cout;
  using std::endl;

  double roll, pitch, yaw, lin_speed;
  double x,y,u,delta;
  bool safe_to_continue;

  x = msg-> pose.pose.position.x;
  y = msg-> pose.pose.position.y;

  // define the quaternion matrix
  tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);

  // convert to rpy
  m.getRPY(roll, pitch, yaw);

  // normalize the speed 
  tf::Vector3 speed = tf::Vector3(msg->twist.twist.linear.x, msg->twist.twist.linear.x, 0.0);
  lin_speed = speed.length();

  cout << "x: " << x << endl;
  cout << "y: " << y << endl;
  cout << "yaw: " << yaw << endl;
  cout << "speed: " << lin_speed << endl;


  u = velocity_msg->velocity;
  delta = angle_msg->steering_angle;

  cout << "u: " << u << endl; 
  cout << "delta: " << delta << endl;

  double state[4] = {x,y,lin_speed,yaw};
  double control_input[2] = {u,delta};


  // create the ros message that will be sent to the VESC


  ackermann_msgs::AckermannDriveStamped ack_msg;
  ack_msg.header.stamp = ros::Time::now();
  ack_msg.drive.steering_angle = delta;
  ack_msg.drive.speed = u;

  
  safe_to_continue= runReachability_bicycle(state, sim_time, walltime, ms, delta, u);
  if (safe_to_continue && !stop)
      ackermann_pub.publish(ack_msg);
  else
  {
    cout << "not safe to continue: " << safe_to_continue << endl; 
    stop = true;
  }
  
}


int main(int argc, char **argv)
{

    // get the path to the file containing the wall points 
    std::string path = ros::package::getPath("rtreach");
    std::string filename = argv[1];
    path = path + "/obstacles/"+filename;
    load_wallpoints(path.c_str(),true);


    using namespace message_filters;

    // initialize the ros node
    ros::init(argc, argv, "reachnode_sync");

    ros::NodeHandle n;
    // ackermann publisher 
    // a description of how the synchronization works can be found here: 
    // http://wiki.ros.org/message_filters/ApproximateTime

    
    
    // define the subscribers you want 
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(n, "racecar/odom", 10);
    message_filters::Subscriber<rtreach::velocity_msg> vel_sub(n, "racecar/velocity_msg", 10);
    message_filters::Subscriber<rtreach::angle_msg> angle_sub(n, "racecar/angle_msg", 10);

    ackermann_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("vesc/ackermann_cmd_mux/input/teleop", 10);
  


    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, rtreach::velocity_msg, rtreach::angle_msg> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, vel_sub,angle_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2,_3));


    while(ros::ok())
    {
      ros::spinOnce();
    }

    // delete the memory allocated to store the wall points
    deallocate_2darr(file_rows,file_columns);


    return 0; 
}