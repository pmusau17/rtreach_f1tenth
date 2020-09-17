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
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


// The following node will receive messages from the LEC which will be prediction of the steering angle
// It will also receive messages from the speed node which will dictate how fast the car should travel
// Initially the assmumption will be the the car moves at constant velocity

extern "C"
{ 
     #include "bicycle_safety.h"
     HyperRectangle runReachability_bicycle_vis(REAL* start, REAL simTime, REAL wallTimeMs, REAL startMs,REAL heading_input, REAL throttle);
     void deallocate_2darr(int rows,int columns);
     void load_wallpoints(const char * filename, bool print);
     HyperRectangle hull;
     void println(HyperRectangle * r);
     void allocate_obstacles(int num_obstacles,double (*points)[2]);
     void deallocate_obstacles(int num_obstacles);
}


/**
* NodeHandle is the main access point to communications with the ROS system.
* The first NodeHandle constructed will fully initialize this node, and the last
* NodeHandle destructed will close down the node.
*/


ros::Publisher ackermann_pub; 
ros::Publisher vis_pub;
ros::Subscriber sub; // markerArray subscriber

// reachability parameters
const double sim_time = 1.0;
double ms = 0.0; // this is redundant will remove in refactoring
const double walltime = 80; // this in ms apparently wtf the declaration doesn't say that 
int markers_allocated = 0;






void callback(const nav_msgs::Odometry::ConstPtr& msg, const rtreach::velocity_msg::ConstPtr& velocity_msg, const rtreach::angle_msg::ConstPtr& angle_msg, const ackermann_msgs::AckermannDriveStamped::ConstPtr& safety_msg)
{
  using std::cout;
  using std::endl;

  double roll, pitch, yaw, lin_speed;
  double x,y,u,delta;
  

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

  cout << "x: " << x;
  cout << " y: " << y;
  cout << " yaw: " << yaw;
  cout << " speed: " << lin_speed << endl;


  u = velocity_msg->velocity;
  delta = angle_msg->steering_angle;

  cout << "u: " << u << endl; 
  cout << "delta: " << delta << endl;

  double state[4] = {x,y,lin_speed,yaw};
  double control_input[2] = {u,delta};


  // create the ros message that will be sent to the VESC

  if(markers_allocated>0)
  {
    ackermann_msgs::AckermannDriveStamped ack_msg;
    
    hull = runReachability_bicycle_vis(state, sim_time, walltime, ms, delta, u);
    println(&hull);

    
    hull.dims[0].min = hull.dims[0].min  - 0.25;
    hull.dims[0].max = hull.dims[0].max  + 0.25;
    hull.dims[1].min = hull.dims[1].min  - 0.15;
    hull.dims[1].max = hull.dims[1].max  + 0.15;
    // publish marker

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    //marker.ns = "my_namespace";
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = (hull.dims[0].max+hull.dims[0].min)/2.0;
    marker.pose.position.y = (hull.dims[1].max+hull.dims[1].min)/2.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = (hull.dims[0].max-hull.dims[0].min);
    marker.scale.y = (hull.dims[1].max-hull.dims[1].min);
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    vis_pub.publish( marker );
  }
  
}

void obstacle_callback(const visualization_msgs::MarkerArray::ConstPtr& marker_msg)
{

     
    std::vector<visualization_msgs::Marker> markers = marker_msg->markers;
    int num_obstacles = markers.size();
    double points[num_obstacles][2]; 
    int i;
    for (i = 0; i< num_obstacles;i++)
    {
      points[i][0] = markers.at(i).pose.position.x;
      points[i][1] = markers.at(i).pose.position.y;
    }

    if(markers_allocated<1)
    {
      allocate_obstacles(num_obstacles,points);
      markers_allocated+=1;
    }
    else
    {
      sub.shutdown();
    }
    std::cout << obstacles[0][0][0] <<", " << obstacles[0][0][1] << std::endl;
}


int main(int argc, char **argv)
{

    // get the path to the file containing the wall points 
    std::string path = ros::package::getPath("rtreach");
    
    if(argv[1] == NULL)
    {
        std::cout << "Please provide the file containing the obstacle locations (i.e porto_obstacles.txt)" << std::endl;
        exit(0);
    }
   
    std::string filename = argv[1];

    path = path + "/obstacles/"+filename;
    load_wallpoints(path.c_str(),true);


    using namespace message_filters;

    // initialize the ros node
    ros::init(argc, argv, "visualize_node");

    ros::NodeHandle n;
    // ackermann publisher 
    // a description of how the synchronization works can be found here: 
    // http://wiki.ros.org/message_filters/ApproximateTime

    
    
    // define the subscribers you want 
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(n, "racecar/odom", 10);
    message_filters::Subscriber<rtreach::velocity_msg> vel_sub(n, "racecar/velocity_msg", 10);
    message_filters::Subscriber<rtreach::angle_msg> angle_sub(n, "racecar/angle_msg", 10);
    message_filters::Subscriber<ackermann_msgs::AckermannDriveStamped> safety_sub(n, "racecar/safety", 1);

    ackermann_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("vesc/ackermann_cmd_mux/input/teleop", 10);
    vis_pub = n.advertise<visualization_msgs::Marker>( "reach_hull", 5 );
  
    sub = n.subscribe("obstacle_locations", 1000, obstacle_callback);

    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, rtreach::velocity_msg, rtreach::angle_msg,ackermann_msgs::AckermannDriveStamped> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), odom_sub, vel_sub,angle_sub,safety_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2,_3,_4));


    while(ros::ok())
    {
      ros::spinOnce();
    }

    // delete the memory allocated to store the wall points
    deallocate_2darr(file_rows,file_columns);
    deallocate_obstacles(obstacle_count);


    return 0; 
}