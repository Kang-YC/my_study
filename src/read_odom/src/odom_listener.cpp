#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include <iostream>
#include <fstream>
#include <tf/transform_datatypes.h>
#include <string>
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Dense>


#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
using namespace std;
using namespace Eigen;
const double PI = 3.1415926;


ofstream myfile1;
ofstream myfile2;
ofstream myfile3;
ofstream myfile4;
ofstream myfile5;
ofstream myfile6;
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
  roll = roll*180 /PI;
  pitch = -pitch*180 /PI;
  yaw = -yaw*180 /PI;
  /*ROS_INFO_NAMED("odomrec", "Seq: %d ", msg->header.seq);
  ROS_INFO_NAMED("odomrec", "Position-> x: %f , y: %f , z: %f ", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  ROS_INFO_NAMED("odomrec", "Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  ROS_INFO_NAMED("odomrec", "Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);*/
  //ROS_INFO_NAMED("odomrec", "Position-> x: %f, y: %f, z: %f ", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  //ROS_INFO_NAMED("odomrec", "Orientation-> roll: %f, pitch: %f, yaw: %f ", roll, pitch, yaw);
  myfile1<< msg-> header.stamp<<",";
  myfile1 <<msg->pose.pose.position.x << "," << msg->pose.pose.position.y << "," << msg->pose.pose.position.z;
  myfile1 <<"," << roll << "," << pitch << "," << yaw ;
 
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



void chatterCallback_odom(const nav_msgs::Odometry::ConstPtr& msg)
{
  
  // geometry_msgs::Quaternion geoQuat = msg->pose.pose.orientation;
  // tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(odom_roll, odom_roll, odom_roll);
  //ROS_INFO_NAMED("Odom", "Orientation-> roll: %f, pitch: %f, yaw: %f ", odom_roll, odom_pitch, odom_yaw);
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  roll = roll*180 /PI;
  pitch = pitch*180 /PI;
  yaw = yaw*180 /PI;
  ROS_DEBUG("odom success");
 
  //myfile3<<"Odom;";
  myfile3<< msg-> header.stamp<<",";
  myfile3 <<msg->pose.pose.position.x<< "," <<msg->pose.pose.position.y<< ","<<msg->pose.pose.position.z;
  myfile3 <<","<< roll << "," << pitch << "," << yaw;
  myfile3 << "\n";
}




void chatterCallback_status(const std_msgs::String::ConstPtr& msg)
{
    //ROS_INFO_NAMED("Odom", "Orientation-> roll: %f, pitch: %f, yaw: %f ", odom_roll, odom_pitch, odom_yaw);
  ROS_DEBUG("status success");
 
 // myfile4<< msg-> header.stamp<<";";
  myfile2<<"Status;";
  myfile2 << msg->data;
  myfile2 << "\n";
}






void chatterCallback_coupled(const nav_msgs::Odometry::ConstPtr& msg)
{
  double roll, pitch, yaw;
  // Eigen::Quaterniond tmp_Q ;
  // tmp_Q={msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z};  
  // Vector3d coupled_euler =  tmp_Q.toRotationMatrix().eulerAngles(0,1,2);
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
 // Vector3d coupled_euler =  tmp_Q.toRotationMatrix().eulerAngles(2,1,0);
  roll = roll*180 /PI;
  pitch = pitch*180 /PI;
  yaw = yaw*180 /PI;

  // roll = coupled_euler[0]*180 /PI;
  // pitch = coupled_euler[1]*180 /PI;
  // yaw = coupled_euler[2]*180 /PI;
  /*ROS_INFO_NAMED("odomrec", "Seq: %d ", msg->header.seq);
  ROS_INFO_NAMED("odomrec", "Position-> x: %f , y: %f , z: %f ", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  ROS_INFO_NAMED("odomrec", "Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  ROS_INFO_NAMED("odomrec", "Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);*/
  //ROS_INFO_NAMED("odomrec", "Position-> x: %f, y: %f, z: %f ", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  //ROS_INFO_NAMED("odomrec", "Orientation-> roll: %f, pitch: %f, yaw: %f ", roll, pitch, yaw);
  myfile4<< msg-> header.stamp<<",";
  myfile4 <<msg->pose.pose.position.x << "," << msg->pose.pose.position.y << "," << msg->pose.pose.position.z;
  myfile4 <<"," <<roll << "," << pitch << "," << yaw;
 
  //myfile << msg->pose.pose.orientation.x << ";" << msg->pose.pose.orientation.y << ";" << msg->pose.pose.orientation.z << ";" << msg->pose.pose.orientation.w;
  //myfile << msg->twist.twist.linear.x,msg->twist.twist.angular.z;
  myfile4<< "\n";
}

void chatterCallback_imupropagate(const nav_msgs::Odometry::ConstPtr& msg)
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
  myfile5<< msg-> header.stamp<<",";
  myfile5 <<msg->pose.pose.position.x << "," << msg->pose.pose.position.y << "," << msg->pose.pose.position.z;
 
  //myfile << msg->pose.pose.orientation.x << ";" << msg->pose.pose.orientation.y << ";" << msg->pose.pose.orientation.z << ";" << msg->pose.pose.orientation.w;
  //myfile << msg->twist.twist.linear.x,msg->twist.twist.angular.z;
  myfile5<< "\n";
}


void chatterCallback_imuupdate(const nav_msgs::Odometry::ConstPtr& msg)
{
// double roll, pitch, yaw;
//   geometry_msgs::Quaternion geoQuat = msg->pose.pose.orientation;
//   tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
  /*ROS_INFO_NAMED("odomrec", "Seq: %d ", msg->header.seq);
  ROS_INFO_NAMED("odomrec", "Position-> x: %f , y: %f , z: %f ", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  ROS_INFO_NAMED("odomrec", "Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  ROS_INFO_NAMED("odomrec", "Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);*/
  //ROS_INFO_NAMED("odomrec", "Position-> x: %f, y: %f, z: %f ", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  //ROS_INFO_NAMED("odomrec", "Orientation-> roll: %f, pitch: %f, yaw: %f ", roll, pitch, yaw);
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  roll = roll*180 /PI;
  pitch = pitch*180 /PI;
  yaw = yaw*180 /PI;
  myfile6<< msg-> header.stamp<<",";
  myfile6 <<msg->pose.pose.position.x << "," << msg->pose.pose.position.y << "," << msg->pose.pose.position.z;
  myfile6 <<"," <<roll << "," << pitch << "," << yaw;
  //myfile << msg->pose.pose.orientation.x << ";" << msg->pose.pose.orientation.y << ";" << msg->pose.pose.orientation.z << ";" << msg->pose.pose.orientation.w;
  //myfile << msg->twist.twist.linear.x,msg->twist.twist.angular.z;
  myfile6<< "\n";
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

	myfile1.open("/home/kang/data/lidar.txt");
  myfile1.precision(10);
  myfile2.open("/home/kang/data/gps.csv");
  myfile2.precision(10);
  myfile3.open("/home/kang/data/odom.txt");
  myfile3.precision(10);
  myfile4.open("/home/kang/data/coupled.txt");
  myfile4.precision(10);
  myfile5.open("/home/kang/data/imu_propagate.txt");
  myfile5.precision(10);
  myfile6.open("/home/kang/data/imu_update.txt");
  myfile6.precision(10);
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
  ros::Subscriber sub_odom = n.subscribe("gps/odom", 1000, chatterCallback_odom);
  ros::Subscriber gps_status = n.subscribe("gps/pos_type", 1000, chatterCallback_status);
  ros::Subscriber sub_coupled = n.subscribe("coupled_odometry", 1000, chatterCallback_coupled); 
  ros::Subscriber sub_imu_propagate = n.subscribe("imu_propagate", 1000, chatterCallback_imupropagate);
  ros::Subscriber sub_imu_update = n.subscribe("imu_update", 1000, chatterCallback_imuupdate);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

	myfile1.close();
  myfile2.close();
  myfile3.close();
  myfile4.close();
  myfile5.close();
  myfile6.close();
  return 0;
}

