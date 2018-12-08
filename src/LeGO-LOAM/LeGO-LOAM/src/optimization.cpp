#include "ros/ros.h"
#include "std_msgs/String.h"

#include "utility.h"
//#include <Eigen/Core>
//#include <Eigen/Dense>
using namespace std;
using namespace Eigen;
class Optimization{
private:
const double optimizationProcessInterval=0.01;
const int WINDOW_SIZE = 10;

Vector3d Ps;
ros::NodeHandle n;

pcl::PointCloud<PointType>::Ptr LaserCloudOri;
pcl::PointCloud<PointType>::Ptr LaserCloudProj;
        
pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;

double timeLaserCloudOri;
double timeLaserCloudProj;
double timeKeyPoses;
double timeKeyPoses6D;
double timeLastProcessing;


bool newLaserCloudOri;
bool newLaserCloudProj;
bool newCloudKeyPoses3D;
bool newCloudKeyPoses6D;

ros::Subscriber subKeyPoses;
ros::Subscriber subKeyPoses6D;
ros::Subscriber subLaserCloudOri;
ros::Subscriber subLaserCloudProj;

PointTypePose thisTransformation;
PointType currentRobotPosPoint;


public:
  Optimization(): 
    n("~")
  {
  subKeyPoses6D = n.subscribe<sensor_msgs::PointCloud2>("/key_pose_origin_6D", 2, &Optimization::keyPoses6DHandler, this);//?
  subKeyPoses = n.subscribe<sensor_msgs::PointCloud2>("/key_pose_origin", 2, &Optimization::keyPosesHandler, this);
  subLaserCloudOri = n.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_ori", 2, &Optimization::laserCloudOriHandler, this);
  subLaserCloudProj = n.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_proj", 2, &Optimization::laserCloudProjHandler, this);
 

  allocateMemory();
  ROS_INFO("init success");
  }

void laserCloudOriHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
        timeLaserCloudOri = msg->header.stamp.toSec();
        LaserCloudOri->clear();
        pcl::fromROSMsg(*msg, *LaserCloudOri);
        newLaserCloudOri = true;
        //ROS_INFO("Ori success");
}

void laserCloudProjHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
        timeLaserCloudProj = msg->header.stamp.toSec();
        LaserCloudProj->clear();
        pcl::fromROSMsg(*msg, *LaserCloudProj);
        newLaserCloudProj = true;
        //ROS_INFO("Proj success");
}

void keyPosesHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
        timeKeyPoses = msg->header.stamp.toSec();
        cloudKeyPoses3D->clear();
        pcl::fromROSMsg(*msg, *cloudKeyPoses3D);
        newCloudKeyPoses3D = true;
        //ROS_INFO("Keypose success %f",cloudKeyPoses3D->points[0].x);
}

void keyPoses6DHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
        timeKeyPoses6D = msg->header.stamp.toSec();
        cloudKeyPoses6D->clear();
        pcl::fromROSMsg(*msg, *cloudKeyPoses6D);
        newCloudKeyPoses6D = true;
        //ROS_INFO("Keypose 6D success %f / %f /%f /%f",cloudKeyPoses6D->points[0].roll,cloudKeyPoses6D->points[0].pitch,cloudKeyPoses6D->points[0].yaw,cloudKeyPoses6D->points[0].x);
}

void allocateMemory(){

        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        LaserCloudOri.reset(new pcl::PointCloud<PointType>());
        LaserCloudProj.reset(new pcl::PointCloud<PointType>());       
}

void currrentPoseProcess()
{

}

void imuProcess()
{

}


void optimizationProcess()
{

}


void run(){

if (newLaserCloudOri  && std::abs(timeLaserCloudOri  - timeKeyPoses) < 0.005 &&
            newLaserCloudProj    && std::abs(timeLaserCloudProj    - timeKeyPoses) < 0.005 &&
            newCloudKeyPoses3D && newCloudKeyPoses6D){
  newLaserCloudOri= false;
  newLaserCloudProj= false;
  newCloudKeyPoses3D= false;
  newCloudKeyPoses6D= false;
  if (timeKeyPoses6D - timeLastProcessing >= optimizationProcessInterval){
    timeLastProcessing=timeKeyPoses6D;

    currrentPoseProcess();
    imuProcess();
    optimizationProcess();


  }

}
}

};

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */




int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "lego_loam");

  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,ros::console::levels::Debug);

  ROS_INFO("\033[1;32m---->\033[0m Lidar-IMU Optimization Started.");
  
  Optimization LIO;
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
 

  ros::Rate rate(200);
  while(ros::ok())
  {
    ros::spinOnce();

    LIO.run();
    
    rate.sleep();
  }


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
 

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */

  return 0;
}