#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include <iostream>
#include <fstream>
#include <tf/transform_datatypes.h>
#include <string>
#include <iomanip>
using namespace std;


ofstream myfile1;
ofstream myfile2;
/**
 * This tutorial demonstrates simple receipt of position and speed of the Evarobot over the ROS system.
 */

/**
 * Callback function executes when new topic data comes.
 * Task of the callback function is to print data to screen.
 */
void chatterCallback_lidar(const nav_msgs::Odometry::ConstPtr& msg)
{
double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = msg->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
  /*ROS_INFO_NAMED("odomrec", "Seq: %d ", msg->header.seq);
  ROS_INFO_NAMED("odomrec", "Position-> x: %f , y: %f , z: %f ", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  ROS_INFO_NAMED("odomrec", "Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  ROS_INFO_NAMED("odomrec", "Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);*/
  //ROS_INFO_NAMED("odomrec", "Position-> x: %f, y: %f, z: %f ", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  //ROS_INFO_NAMED("odomrec", "Orientation-> roll: %f, pitch: %f, yaw: %f ", roll, pitch, yaw);
  myfile1<< msg-> header.stamp<<";";
	myfile1 <<msg->pose.pose.position.x << ";" << msg->pose.pose.position.y << ";" << msg->pose.pose.position.z<<";";
        myfile1 << roll << ";" << pitch << ";" <<yaw;
	//myfile << msg->pose.pose.orientation.x << ";" << msg->pose.pose.orientation.y << ";" << msg->pose.pose.orientation.z << ";" << msg->pose.pose.orientation.w;
	//myfile << msg->twist.twist.linear.x,msg->twist.twist.angular.z;
	myfile1<< "\n";
}

void chatterCallback_gps(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  float latitude, longitude;

  latitude=msg->latitude;
  longitude=msg->longitude;

  //ROS_INFO_NAMED("gps", "latitude: %f,longitude: %f", msg->latitude,msg->longitude);
  
  myfile2<< msg-> header.stamp<<";";
  myfile2 << msg->latitude<< ";" <<msg->longitude;
  myfile2 << "\n";
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "ros_odom_listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

	myfile1.open("/home/kang/data/lidar.csv");
  myfile1.precision(10);
  myfile2.open("/home/kang/data/gps.csv");
  myfile2.precision(10);
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

  //ros::Subscriber sub = n.subscribe("gps/odom", 1000, chatterCallback);
  ros::Subscriber sub_lidar = n.subscribe("integrated_to_init", 1000, chatterCallback_lidar);
  ros::Subscriber sub_gps = n.subscribe("gps/fix", 1000, chatterCallback_gps);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

	myfile1.close();
  myfile2.close();
  return 0;
}

