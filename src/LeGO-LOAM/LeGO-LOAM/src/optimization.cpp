#include "ros/ros.h"
#include "std_msgs/String.h"

#include "utility.h"
#include <ceres/local_parameterization.h>
#include <ceres/autodiff_local_parameterization.h>
#include <ceres/types.h>
#include <ceres/rotation.h>
#include <ceres/ceres.h>

#include <ceres/loss_function.h>
#include "CPUTimer.h"
#include "icp-ceres.h"
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;




class Optimization{
private:
const double optimizationProcessInterval=0.1;

static const int WINDOW_SIZE = 10;
static const int SIZE_POSE = 7;
static const int SIZE_SPEEDBIAS = 9;
ros::NodeHandle n;

pcl::PointCloud<PointType>::Ptr LaserCloudOri;
pcl::PointCloud<PointType>::Ptr LaserCloudProj;
        
pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;

deque<pcl::PointCloud<PointType>::Ptr> CloudKeyFramesOri;//
deque<pcl::PointCloud<PointType>::Ptr> CloudKeyFramesProj;//
vector<int> CloudKeyPosesID;

int laserCloudOriNum;
int laserCloudProjNum;

double timeLaserCloudOri;
double timeLaserCloudProj;
double timeKeyPoses;
double timeKeyPoses6D;
double timeLastProcessing;


bool newLaserCloudOri;
bool newLaserCloudProj;
bool newCloudKeyPoses3D;
bool newCloudKeyPoses6D;

bool smallProblem=false;

ros::Subscriber subKeyPoses;
ros::Subscriber subKeyPoses6D;
ros::Subscriber subLaserCloudOri;
ros::Subscriber subLaserCloudProj;
ros::Subscriber subImu;

ros::Publisher pubCoupledOdometry;

nav_msgs::Odometry odomCoupled;
tf::StampedTransform aftCoupledTrans;
tf::TransformBroadcaster tfBroadcaster;

Eigen::Matrix3d curRx;
Eigen::Matrix3d curRx_inverse;
Eigen::Vector3d cur_t;


Vector3d Ps[(WINDOW_SIZE + 1)];
Vector3d Vs[(WINDOW_SIZE + 1)];
Matrix3d Rs[(WINDOW_SIZE + 1)];
Vector3d Bas[(WINDOW_SIZE + 1)];
Vector3d Bgs[(WINDOW_SIZE + 1)];

double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];


int frame_count;

std_msgs::Header lidarHeader;


public:
  Optimization(): 
    n("~")
  {
  subKeyPoses6D = n.subscribe<sensor_msgs::PointCloud2>("/key_pose_origin_6D", 5, &Optimization::keyPoses6DHandler, this);//?
  subKeyPoses = n.subscribe<sensor_msgs::PointCloud2>("/key_pose_origin", 5, &Optimization::keyPosesHandler, this);
  subLaserCloudOri = n.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_ori", 5, &Optimization::laserCloudOriHandler, this);
  subLaserCloudProj = n.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_proj", 5, &Optimization::laserCloudProjHandler, this);
  subImu = n.subscribe<sensor_msgs::Imu> (imuTopic, 1000, &Optimization::imuHandler, this);

  pubCoupledOdometry = n.advertise<nav_msgs::Odometry>("/coupled_odometry", 1000);

  

  aftCoupledTrans.frame_id_ = "/camera_init";
  aftCoupledTrans.child_frame_id_ = "/aft_coupled";

  odomCoupled.header.frame_id = "/camera_init";
  odomCoupled.child_frame_id = "/aft_coupled";


  allocateMemory();
  clearState();
  ROS_INFO("init success");
  }


void laserCloudOriHandler(const sensor_msgs::PointCloud2ConstPtr& msg)
{
        lidarHeader = msg->header;
        timeLaserCloudOri = msg->header.stamp.toSec();
        LaserCloudOri->clear();
        pcl::fromROSMsg(*msg, *LaserCloudOri);
        newLaserCloudOri = true;
        laserCloudOriNum=LaserCloudOri->points.size();
        //ROS_DEBUG("Ori sub Num %d",laserCloudOriNum);
}


void laserCloudProjHandler(const sensor_msgs::PointCloud2ConstPtr& msg)
{
        timeLaserCloudProj = msg->header.stamp.toSec();
        LaserCloudProj->clear();
        pcl::fromROSMsg(*msg, *LaserCloudProj);
        newLaserCloudProj = true;
        laserCloudProjNum=LaserCloudProj->points.size();
        //ROS_INFO("Proj Num %d",laserCloudProjNum);
}


void keyPosesHandler(const sensor_msgs::PointCloud2ConstPtr& msg)
{
        timeKeyPoses = msg->header.stamp.toSec();
        cloudKeyPoses3D->clear();
        pcl::fromROSMsg(*msg, *cloudKeyPoses3D);
        newCloudKeyPoses3D = true;
        int numPoses = cloudKeyPoses3D->points.size();
        ROS_DEBUG("numPoses sub %d", numPoses);
        //ROS_INFO("Keypose success %f",cloudKeyPoses3D->points[0].x);
}


void keyPoses6DHandler(const sensor_msgs::PointCloud2ConstPtr& msg)
{
        timeKeyPoses6D = msg->header.stamp.toSec();
        cloudKeyPoses6D->clear();
        pcl::fromROSMsg(*msg, *cloudKeyPoses6D);
        newCloudKeyPoses6D = true;
        //ROS_INFO("Keypose 6D success %f / %f /%f /%f",cloudKeyPoses6D->points[0].roll,cloudKeyPoses6D->points[0].pitch,cloudKeyPoses6D->points[0].yaw,cloudKeyPoses6D->points[0].x);
}


 void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{
        

        float accX = imuIn->linear_acceleration.x ;
        float accY = imuIn->linear_acceleration.y ;
        float accZ = imuIn->linear_acceleration.z ;

        
}


void allocateMemory()
{

        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        LaserCloudOri.reset(new pcl::PointCloud<PointType>());
        LaserCloudProj.reset(new pcl::PointCloud<PointType>());       
}



void clearState()
{
  for (int i = 0; i < WINDOW_SIZE + 1; i++)
  {
    Rs[i].setIdentity();
    Ps[i].setZero();
    Vs[i].setZero();
    Bas[i].setZero();
    Bgs[i].setZero();
  }
  frame_count=0;
  ROS_INFO("clear success");

}



ceres::Solver::Options getOptions()
{
    // Set a few options
    ceres::Solver::Options options;
    //options.use_nonmonotonic_steps = true;
    //options.preconditioner_type = ceres::IDENTITY;
    options.linear_solver_type = ceres::DENSE_QR;
    
    options.max_num_iterations = 50;

//    options.preconditioner_type = ceres::SCHUR_JACOBI;
//    options.linear_solver_type = ceres::DENSE_SCHUR;
//    options.use_explicit_schur_complement=true;
//    options.max_num_iterations = 100;

    cout << "Ceres Solver getOptions()" << endl;
    cout << "Ceres preconditioner type: " << options.preconditioner_type << endl;
    cout << "Ceres linear algebra type: " << options.sparse_linear_algebra_library_type << endl;
    cout << "Ceres linear solver type: " << options.linear_solver_type << endl;

    return options;
}


ceres::Solver::Options getOptionsMedium()
{
    // Set a few options
    ceres::Solver::Options options;

    #ifdef _WIN32
        options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
        options.linear_solver_type                 = ceres::ITERATIVE_SCHUR;
        options.preconditioner_type                = ceres::SCHUR_JACOBI;
    #else
        //options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    #endif // _WIN32

    //If you are solving small to medium sized problems, consider setting Solver::Options::use_explicit_schur_complement to true, it can result in a substantial performance boost.
    options.use_explicit_schur_complement=true;
    options.max_num_iterations = 50;

    cout << "Ceres Solver getOptionsMedium()" << endl;
    cout << "Ceres preconditioner type: " << options.preconditioner_type << endl;
    cout << "Ceres linear algebra type: " << options.sparse_linear_algebra_library_type << endl;
    cout << "Ceres linear solver type: " << options.linear_solver_type << endl;

    return options;
}


void solveProblem(ceres::Problem &problem)
{

  ROS_DEBUG("solveProblem %d", frame_count+1);
    ceres::Solver::Summary summary;
    ceres::Solve(smallProblem ? getOptions() : getOptionsMedium(), &problem, &summary);
    if(!smallProblem) std::cout << "Final report:\n" << summary.FullReport();
}




void currrentPoseProcess()
{

  PointTypePose currentRobotPos6D;
  PointType currentRobotPos3D;
  double roll,pitch,yaw,x,y,z;

  int numPoses = cloudKeyPoses3D->points.size();
  ROS_DEBUG("currrentPoseProcess  %d", numPoses);
  if(numPoses==0)
    return;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               

  int currentInd    = numPoses-1;
  currentRobotPos6D =cloudKeyPoses6D->points[currentInd];

////initial the pose  
  roll  = currentRobotPos6D.yaw;//to Cartesian coordinate system
  pitch = currentRobotPos6D.roll;
  yaw   = currentRobotPos6D.pitch;
  x     = currentRobotPos6D.z;
  y     = currentRobotPos6D.x;
  z     = currentRobotPos6D.y;

  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = rollAngle * pitchAngle *  yawAngle;//!!! indicate the orger of rotate

  curRx         = q.toRotationMatrix();//from world frame (ypr)
  //curRx_inverse = curRx.inverse();
  cur_t<< x,y,z ;


  
  if(currentInd < WINDOW_SIZE){
    Ps[currentInd] = cur_t;
    Rs[currentInd] = curRx;
  }
  else
  {
     Ps[WINDOW_SIZE] = cur_t;
     Rs[WINDOW_SIZE] = curRx;

  }



  //ROS_INFO("frame_count %d", frame_count);
  //ROS_INFO("currentInd %d", currentInd);

 //int thisKeyInd = (int)cloudKeyPoses6D->points[j].intensity;
 //CloudKeyPosesID.   push_back(currentInd);
 CloudKeyFramesOri. push_back(LaserCloudOri);
 CloudKeyFramesProj.push_back(LaserCloudProj);   

 // if(currentInd >= WINDOW_SIZE)
 // {
 //  CloudKeyFramesOri.pop_front();
 //  CloudKeyFramesProj.pop_front();
 // }              


  frame_count=currentInd;// different with thiskeyid??

 }


void imuProcess()
{

}


void optimizationProcess()
{
  if(frame_count >= WINDOW_SIZE)
  {

    solveOdometry();
    slideWindow();

  }
 

  

}

void solveOdometry()
{
  
  ROS_DEBUG("solveOdometry %d", frame_count+1);
  ceres::Problem problem;
  ceres::LossFunction *loss_function = new ceres::CauchyLoss(1);
  vector2double();

  for(int i=0; i<WINDOW_SIZE;i++)
  {

    int j   = i+1;
    int ind = i ;

    for(int k=0;k<CloudKeyFramesOri[ind]->points.size();k++)
    {
      
      Vector3d ori, proj;

      //ori in body frame
      //proj in world frame

      ori<< CloudKeyFramesOri[ind]->points[k].z , CloudKeyFramesOri[ind]->points[k].x , CloudKeyFramesOri[ind]->points[k].y;
      
      proj<< CloudKeyFramesProj[ind]->points[k].z, CloudKeyFramesProj[ind]->points[k].x , CloudKeyFramesProj[ind]->points[k].y;

      // test the rotation
      // double point[3] = {ori[0], ori[1], ori[2]};
      // double q[4] = {para_Pose[ind][6],para_Pose[ind][3],para_Pose[ind][4],para_Pose[ind][5]};//!!!!!w x y z 
      // double p[3];


      // ceres::QuaternionRotatePoint( q, point, p);
      
      //   p[0] += para_Pose[ind][0];
      //   p[1] += para_Pose[ind][1];
      //   p[2] += para_Pose[ind][2];
      // cout << "ori"<<p[0]<<" " <<p[1]<<" "<<p[2]<< endl;
      // cout << proj << endl;
     
    
      //ROS_DEBUG("ori x y  %f %f", ori[0],ori[1]);
      if(ori[0] > 50 ||  ori[1] > 50  || ori[2] > 50)
        continue;

      
      ceres::CostFunction* lidar_cost_function = ICPCostFunctions::PointToPointError_EigenQuaternion::Create(proj,ori);
      
      problem.AddResidualBlock(lidar_cost_function, loss_function, para_Pose[ind]);
      
  


     // PtestCeres2 = ICP_Ceres::pointToPoint_EigenQuaternion(ori,proj);

    }
  }
  solveProblem(problem);
  double2vector();



}

void vector2double(){
  for (int i = 0; i < WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();

        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();

        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }

}


void double2vector()
{
    for (int i = 0; i < WINDOW_SIZE; i++)
    {

        Rs[i] =  Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
        
        Ps[i] =  Vector3d(para_Pose[i][0], para_Pose[i][1] ,para_Pose[i][2]);

        

        Vs[i] =  Vector3d(para_SpeedBias[i][0],
                                    para_SpeedBias[i][1],
                                    para_SpeedBias[i][2]);

        Bas[i] = Vector3d(para_SpeedBias[i][3],
                          para_SpeedBias[i][4],
                          para_SpeedBias[i][5]);

        Bgs[i] = Vector3d(para_SpeedBias[i][6],
                          para_SpeedBias[i][7],
                          para_SpeedBias[i][8]);
    }
    ROS_DEBUG("P %f",Ps[WINDOW_SIZE-1][0]);
}



void slideWindow()
{

  
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
      Rs[i].swap(Rs[i + 1]);
      //Headers[i] = Headers[i + 1];
      Ps[i].swap(Ps[i + 1]);
      Vs[i].swap(Vs[i + 1]);
      Bas[i].swap(Bas[i + 1]);
      Bgs[i].swap(Bgs[i + 1]);
    }// destory the front values

 
    CloudKeyFramesOri.pop_front();
    CloudKeyFramesProj.pop_front();
 


  
  ROS_DEBUG("slideWindow %d", frame_count+1);
}


void pubOdometry(){
  

  
  //nav_msgs::Odometry odomCoupled;
  //odometry.header = lidarHeader;
  odomCoupled.header.stamp = ros::Time().fromSec(timeKeyPoses);
  odomCoupled.header.frame_id = "camera_init";
  odomCoupled.child_frame_id = "aft_coupled";
  Quaterniond tmp_Q;
  tmp_Q = Quaterniond(Rs[WINDOW_SIZE]);
  odomCoupled.pose.pose.position.x = Ps[WINDOW_SIZE-1].x();
  odomCoupled.pose.pose.position.y = Ps[WINDOW_SIZE-1].y();
  odomCoupled.pose.pose.position.z = Ps[WINDOW_SIZE-1].z();
  odomCoupled.pose.pose.orientation.x = tmp_Q.x();
  odomCoupled.pose.pose.orientation.y = tmp_Q.y();
  odomCoupled.pose.pose.orientation.z = tmp_Q.z();
  odomCoupled.pose.pose.orientation.w = tmp_Q.w();
  odomCoupled.twist.twist.linear.x = Vs[WINDOW_SIZE].x();
  odomCoupled.twist.twist.linear.y = Vs[WINDOW_SIZE].y();
  odomCoupled.twist.twist.linear.z = Vs[WINDOW_SIZE].z();
  pubCoupledOdometry.publish(odomCoupled);

  aftCoupledTrans.stamp_ = ros::Time().fromSec(timeKeyPoses);
  aftCoupledTrans.setRotation(tf::Quaternion(tmp_Q.x(), tmp_Q.y(),tmp_Q.z(), tmp_Q.w()));
  aftCoupledTrans.setOrigin(tf::Vector3(Ps[WINDOW_SIZE].x(), Ps[WINDOW_SIZE].y(), Ps[WINDOW_SIZE].z()));
  tfBroadcaster.sendTransform(aftCoupledTrans);
  

  ROS_DEBUG("pubOdometry %d %f", frame_count+1, Ps[WINDOW_SIZE-1].x());

}



void run(){

  if (newLaserCloudOri  && std::abs(timeLaserCloudOri  - timeKeyPoses) < 0.005 &&
    newLaserCloudProj && std::abs(timeLaserCloudProj - timeKeyPoses) < 0.005 &&
     newCloudKeyPoses3D && newCloudKeyPoses6D){
  newLaserCloudOri   = false;
  newLaserCloudProj  = false;
  newCloudKeyPoses3D = false;
  newCloudKeyPoses6D = false;
    
    if (timeKeyPoses6D - timeLastProcessing >= optimizationProcessInterval){
    timeLastProcessing=timeKeyPoses6D;

    currrentPoseProcess();

    //currrentPointProcess();

    imuProcess();

    optimizationProcess();

    pubOdometry();
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
 

  ros::Rate rate(100);
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