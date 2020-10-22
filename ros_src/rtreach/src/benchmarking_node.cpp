#include "ros/ros.h"
#include <iostream>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <rtreach/angle_msg.h>
#include <rtreach/velocity_msg.h>
#include <rtreach/stamped_ttc.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/package.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include<ctime>
#include <fstream>
#include <string>

// The following node will receive messages from the LEC which will be prediction of the steering angle
// It will also receive messages from the speed node which will dictate how fast the car should travel
// Initially the assmumption will be the the car moves at constant velocity

extern "C"
{ 
     #include "bicycle_safety.h"
     bool runReachability_bicycle(double * start, double  simTime, double  wallTimeMs, double  startMs,double  heading_input, double  throttle);
     void deallocate_2darr(int rows,int columns);
     void load_wallpoints(const char * filename, bool print);
     void allocate_obstacles(int num_obstacles,double (*points)[2]);
     void deallocate_obstacles(int num_obstacles);
}


/**
* NodeHandle is the main access point to communications with the ROS system.
* The first NodeHandle constructed will fully initialize this node, and the last
* NodeHandle destructed will close down the node.
*/


ros::Publisher ackermann_pub; // control command publisher
ros::Subscriber sub; // markerArray subscriber

// reachability parameters
double sim_time = 0.5;
const double walltime = 25; // 25 ms corresponds to 40 hz
double ttc = 0.0;
int markers_allocated = 0;
bool stop = false;
int safePeriods =0;


// variables added to deal with timing 
double time_taken_lec = 0.0;
double time_taken_safety_controller = 0.0;
bool prev_stop = false;
bool switched = false;
clock_t start, end,total_start,total_end, reach_start, reach_end;
int begin_count = 0;
double wcet = 0.0;


// variable for cumulative moving average
double count = 0.0;
double avg_reach_time = 0.0; 
double new_mean;
double differential;


void callback(const nav_msgs::Odometry::ConstPtr& msg, const rtreach::velocity_msg::ConstPtr& velocity_msg, const rtreach::angle_msg::ConstPtr& angle_msg, const ackermann_msgs::AckermannDriveStamped::ConstPtr& safety_msg, const rtreach::stamped_ttc::ConstPtr& ttc_msg)
{
  using std::cout;
  using std::endl;

  double roll, pitch, yaw, lin_speed;
  double x,y,u,delta;
  bool safe_to_continue;
  ttc = ttc_msg->ttc;
  
  // the lookahead time should be dictated by the lookahead time
  // since the car is moving at 1 m/s the max sim time is 1.5 seconds
  sim_time = fmin(1.5*ttc,0.5);
  // std::cout << "sim_time: " << sim_time << endl;

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

  u = velocity_msg->velocity;
  delta = angle_msg->steering_angle;

  // cout << "u: " << u << endl; 
  // cout << "delta: " << delta << endl;

  double state[4] = {x,y,lin_speed,yaw};
  double control_input[2] = {u,delta};


  // create the ros message that will be sent to the VESC

  if(markers_allocated>0)
  {
    ackermann_msgs::AckermannDriveStamped ack_msg;
    if(begin_count<1)
    {
        start = clock();
        total_start = start;
        begin_count++;
    }
        
  
    if(stop && safePeriods>30)
    {
      stop = false;
    }
    else if(safety_msg->drive.speed == 0.0)
    {
      stop = true;
      safePeriods=0;
      // ROS_WARN("Safety controller issuing stop command.");
      
    }
    reach_start = clock();
    safe_to_continue= runReachability_bicycle(state, sim_time, walltime, 0.0, delta, u);
    reach_end = clock();

    double reach_time = double(reach_end - reach_start) / double(CLOCKS_PER_SEC);
    cout << "reach_time: " << reach_time << endl;
    if(reach_time>wcet)
    {
      wcet = reach_time;
    }

    // calculate exponential moving average
    count++;
    differential = (reach_time - avg_reach_time) / count;
    new_mean = avg_reach_time + differential;
    avg_reach_time = new_mean;


    // detect switches of safe to non-safe
    if(stop != prev_stop)
    {
        switched = true; 
    }    
    
    // store current decision    
    if(switched)
    {
        ROS_WARN("Switch Occured,Switch Occured,Switch Occured,Switch Occured...");
        switched = false;
        // reset the clock to start logging the new controller
        end = clock();
        double time_taken = double(end - start) / double(CLOCKS_PER_SEC);
        if(!stop)
        {
            time_taken_safety_controller += time_taken;
        }
        else
        {
            time_taken_lec = time_taken;
        }
        // cout << "time taken: " << time_taken << endl;
        start = clock();
       
    }
    
        



    // main loop
    if (safe_to_continue && !stop)
    {
        ack_msg.drive.steering_angle = delta;
        ack_msg.drive.speed = u;
        ack_msg.header.stamp = ros::Time::now();
        ackermann_pub.publish(ack_msg);
    }
        
    else
    {
        // cout << "safe: angle: " << safety_msg->drive.steering_angle << " safe speed: " << safety_msg->drive.speed << endl;
        ack_msg.drive.steering_angle = safety_msg->drive.steering_angle;
        ack_msg.drive.speed = safety_msg->drive.speed;
        ack_msg.header.stamp = ros::Time::now();
        ackermann_pub.publish(ack_msg);
        ROS_WARN("using safety controller.");
        cout << "using safety controller...safe?: " << safe_to_continue << endl; 
        stop = true;
    }

    if(stop & safe_to_continue)
    {
      safePeriods+=1;
    }
    else
    {
      safePeriods = 0;
    }
    prev_stop = stop;
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
    ros::init(argc, argv, "reachnode_sync");
    std::cout << "sleeping" << std::endl;
    sleep(10);
    std::cout << "done sleeping" << std:: endl;
    ros::NodeHandle n;
    // ackermann publisher 
    // a description of how the synchronization works can be found here: 
    // http://wiki.ros.org/message_filters/ApproximateTime

    
    
    // define the subscribers you want 
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(n, "racecar/odom", 10);
    message_filters::Subscriber<rtreach::velocity_msg> vel_sub(n, "racecar/velocity_msg", 10);
    message_filters::Subscriber<rtreach::angle_msg> angle_sub(n, "racecar/angle_msg", 10);
    message_filters::Subscriber<ackermann_msgs::AckermannDriveStamped> safety_sub(n, "racecar/safety", 10);
    message_filters::Subscriber<rtreach::stamped_ttc> ttc_sub(n, "racecar/ttc", 10);

    ackermann_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("vesc/ackermann_cmd_mux/input/teleop", 10);
  
    sub = n.subscribe("obstacle_locations", 1000, obstacle_callback);


    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, rtreach::velocity_msg, rtreach::angle_msg,ackermann_msgs::AckermannDriveStamped,rtreach::stamped_ttc> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), odom_sub, vel_sub,angle_sub,safety_sub,ttc_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2,_3,_4,_5));

    ros::Rate r(40); // 20 hz
    while(ros::ok())
    {
      ros::spinOnce();
      r.sleep();
    }

    // delete the memory allocated to store the wall points
    deallocate_2darr(file_rows,file_columns);
    deallocate_obstacles(obstacle_count);
    
    // complete time logging
    end = clock();
    total_end = end; 
    double time_taken = double(end - start) / double(CLOCKS_PER_SEC);
    double total_time_taken = double(total_end - total_start) / double(CLOCKS_PER_SEC);

    if(stop)
    {
        time_taken_safety_controller += time_taken;
    }
    else
    {
        time_taken_lec = time_taken;
    }

    // declaring argument of time() 
    time_t curr_time;
	  tm * curr_tm;
	  char time_string[100];
    time(&curr_time);
	  curr_tm = localtime(&curr_time);
	  strftime(time_string, 50, "%d/%m/%Y/%T", curr_tm);
	
    path = ros::package::getPath("rtreach")+"/benchmarking/"+"benchmark_experiments.txt";
    std::ofstream outfile(path.c_str() , std::ios::app);
    outfile << time_string << "," << time_taken_lec << "," << time_taken_safety_controller << 
        ","<< total_time_taken << "," << wcet << "," << avg_reach_time << "\n";
    outfile.close();
    
    return 0; 
}