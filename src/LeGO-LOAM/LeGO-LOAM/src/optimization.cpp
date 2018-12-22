#include "ros/ros.h"
#include "std_msgs/String.h"
#include <queue>
#include <stdio.h>

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

#include "utility_rotation.h"
#include "pose_local_parameterization.h"
#include "factor/imu_factor.h"

#include <mutex>

#include <condition_variable>

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
ros::Publisher pubImuPropogate;

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

bool init_imu = 1;
bool first_imu;
double timeLastIMU = 0;
double latest_time;
double current_time = -1;

Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
Vector3d G = {0, 0, 9.81};

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloud2ConstPtr> ori_buf;

double td;

IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
IntegrationBase *tmp_pre_integration;

vector<double> dt_buf[(WINDOW_SIZE + 1)];
vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

std::mutex m_buf;//m_buf

std::condition_variable con;


public:
  Optimization(): 
    n("~")
  {
  subKeyPoses6D = n.subscribe<sensor_msgs::PointCloud2>("/key_pose_origin_6D", 5, &Optimization::keyPoses6DHandler, this);//?
  subKeyPoses = n.subscribe<sensor_msgs::PointCloud2>("/key_pose_origin", 5, &Optimization::keyPosesHandler, this);
  subLaserCloudOri = n.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_ori", 5, &Optimization::laserCloudOriHandler, this);
  subLaserCloudProj = n.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_proj", 5, &Optimization::laserCloudProjHandler, this);
  subImu = n.subscribe<sensor_msgs::Imu> (imuTopic, 1000, &Optimization::imuHandler, this ,ros::TransportHints().tcpNoDelay());

  pubCoupledOdometry = n.advertise<nav_msgs::Odometry>("/coupled_odometry", 1000);
  //pubImuPropogate = n.advertise<nav_msgs::Odometry>("/imu_propagate", 1000);

  

  aftCoupledTrans.frame_id_ = "/camera_init";
  aftCoupledTrans.child_frame_id_ = "/aft_coupled";

  odomCoupled.header.frame_id = "/camera_init";
  odomCoupled.child_frame_id = "/aft_coupled";


  allocateMemory();
  clearState();
  ROS_INFO("init success");
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
    dt_buf[i].clear();
    linear_acceleration_buf[i].clear();
    angular_velocity_buf[i].clear();
    // if (pre_integrations[i] != nullptr)
    // {
    //   delete pre_integrations[i];
    // }
    // pre_integrations[i] = nullptr;

  }

    // if (tmp_pre_integration != nullptr)
    //   delete tmp_pre_integration;
  

  tmp_P.setZero();
  //tmp_Q.setIdentity();
  tmp_V.setZero();
  tmp_Ba.setZero();
  tmp_Bg.setZero();


  acc_0.setZero();
  gyr_0.setZero();

  first_imu = false;
 

  frame_count=0;
  ROS_INFO("clear success");

}

void laserCloudOriHandler(const sensor_msgs::PointCloud2ConstPtr& msg)
{

        lidarHeader = msg->header;
        timeLaserCloudOri = msg->header.stamp.toSec();
        LaserCloudOri->clear();
        pcl::fromROSMsg(*msg, *LaserCloudOri);
        newLaserCloudOri = true;
        laserCloudOriNum=LaserCloudOri->points.size();

        //m_buf.lock();
        ori_buf.push(msg);
        //m_buf.unlock();
       // con.notify_one();


      // ROS_DEBUG("Ori sub Num %d",laserCloudOriNum);
        ROS_DEBUG("timeLaserCloudOri %f", timeLaserCloudOri);
}


void laserCloudProjHandler(const sensor_msgs::PointCloud2ConstPtr& msg)
{
        timeLaserCloudProj = msg->header.stamp.toSec();
        LaserCloudProj->clear();
        pcl::fromROSMsg(*msg, *LaserCloudProj);
        newLaserCloudProj = true;
        laserCloudProjNum=LaserCloudProj->points.size();
      //  ROS_INFO("Proj Num %d",laserCloudProjNum);
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
  if (imuIn->header.stamp.toSec() <= timeLastIMU)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }

    //m_buf.lock();
    imu_buf.push(imuIn);
    //m_buf.unlock();
//con.notify_one();

    timeLastIMU = imuIn->header.stamp.toSec();
    ROS_DEBUG("timeLastIMU %f", timeLastIMU);
    
    predict(imuIn);
    std_msgs::Header imu_header = imuIn->header;
    imu_header.frame_id = "camera_init";
    pubLatestOdometry(tmp_P, tmp_Q, tmp_V, imu_header);
        
}


void predict(const sensor_msgs::ImuConstPtr &imu_msg)// propogate
{
    double t = imu_msg->header.stamp.toSec();
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - G;//世界坐标系下 上一时刻的 un_acc_0为车加速度（消除了重力） acc_0为读到的
    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;//角速度 un_gyr为车的
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);//q=q*delta(w*t)

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - G;//当前时刻加速度

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);//中值 上一时刻加速度和当前时刻计算

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;//预测位置 p=p+vt+0.5*a*t2
    tmp_V = tmp_V + dt * un_acc;//预测速度 v=v+a*t

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
    //ROS_DEBUG("tmp_P %f", tmp_P.x());
}

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, const std_msgs::Header &header)
{
    Eigen::Quaterniond quadrotor_Q = Q ;

    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "camera_init";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = quadrotor_Q.x();
    odometry.pose.pose.orientation.y = quadrotor_Q.y();
    odometry.pose.pose.orientation.z = quadrotor_Q.z();
    odometry.pose.pose.orientation.w = quadrotor_Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    pubImuPropogate.publish(odometry);

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

  ROS_DEBUG("solveProblem %d", frame_count);
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

  int currentInd    = numPoses-1;
  currentRobotPos6D =cloudKeyPoses6D->points[currentInd];


  //frame_count=numPoses;

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
  cur_t<< x,y,z ;

  
  if(frame_count <= WINDOW_SIZE){// framecount==index 0-9
    Ps[frame_count] = cur_t;
    Rs[frame_count] = curRx;
   // ROS_DEBUG("Ps[currentInd] %f %f %f  ",Ps[currentInd][0], Ps[currentInd][1], Ps[currentInd][2]);
  }
  else
  {
     Ps[WINDOW_SIZE] = cur_t;
     Rs[WINDOW_SIZE] = curRx;
   // ROS_DEBUG("Ps[WINDOW_SIZE] %f %f %f  ",Ps[WINDOW_SIZE][0], Ps[WINDOW_SIZE][1], Ps[WINDOW_SIZE][2]);
  }


  //ROS_INFO("frame_count %d", frame_count);
  //ROS_INFO("currentInd %d", currentInd);

 //int thisKeyInd = (int)cloudKeyPoses6D->points[j].intensity;
 //CloudKeyPosesID.   push_back(currentInd);
 
  pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
  pcl::copyPointCloud(*LaserCloudOri,  *thisCornerKeyFrame);
  pcl::copyPointCloud(*LaserCloudProj,  *thisSurfKeyFrame);
        
  CloudKeyFramesOri. push_back(thisCornerKeyFrame);
  CloudKeyFramesProj.push_back(thisSurfKeyFrame);   
  //ROS_DEBUG("ori size %d",CloudKeyFramesProj.size());

 // if(currentInd >= WINDOW_SIZE)
 // {
 //  CloudKeyFramesOri.pop_front();
 //  CloudKeyFramesProj.pop_front();
 // }              

  
 //  if(frame_count >= WINDOW_SIZE){


 //    float ctRoll = cos(roll);
 //  float stRoll = sin(roll);

 //  float ctPitch = cos(pitch);
 //  float stPitch = sin(pitch);

 //  float ctYaw = cos(yaw);
 //  float stYaw = sin(yaw);

 //  float tInX = x;
 //  float tInY = y;
 //  float tInZ = z;

 //  vector2double();
 //  // for(int kke=0; kke < CloudKeyFramesOri.size();kke++)
 //  //   {
 //  //   ROS_DEBUG("ori every frame %f %f %f  ",CloudKeyFramesOri.at(kke)->points[1].z , CloudKeyFramesOri.at(kke)->points[1].y);
 //  // }

 //  int ind = 2;

 //    for(int k=0;k < CloudKeyFramesOri[ind]->points.size();k++)
 //    {
      
 //      Vector3d ori, proj;

 //      //ori in body frame
 //      //proj in world frame

 //      ori<< CloudKeyFramesOri[ind]->points[k].z , CloudKeyFramesOri[ind]->points[k].x , CloudKeyFramesOri[ind]->points[k].y;
 //      //ROS_DEBUG("ori000 %f %f %f  ",ori[0], ori[1], ori[2]);
 //      proj<< CloudKeyFramesProj[ind]->points[k].z, CloudKeyFramesProj[ind]->points[k].x , CloudKeyFramesProj[ind]->points[k].y;

 //      // test the rotation
 //      if(k < 3 ){
 //      double point[3] = {ori[0], ori[1], ori[2]};
 //      double q0[4] = {para_Pose[ind][6],para_Pose[ind][3],para_Pose[ind][4],para_Pose[ind][5]};//!!!!!w x y z 
 //      double p[3];


 //      ROS_DEBUG("q[4]  %f %f %f",q0[0],q0[1],q0[2]);


 //     ROS_DEBUG("para_Pose0 1 2  %f %f %f",para_Pose[ind][0],para_Pose[ind][1],para_Pose[ind][2]);


 //      ceres::QuaternionRotatePoint( q0, point, p);
      
 //        p[0] += para_Pose[ind][0];
 //        p[1] += para_Pose[ind][1];
 //        p[2] += para_Pose[ind][2];
 //      cout << "ori1"<<p[0]<<" " <<p[1]<<" "<<p[2]<< endl;
 //      cout << "pro1"<<proj[0]<<" " << proj[1]<<" "<<proj[2]<<endl;

 //      }
 //      else 
 //        break;
 //    }


      
 //    for(int i = 0;i<3;i++){

 //            // PointType pointFrom = LaserCloudOri->points[i];
 //            // PointType pointTo = LaserCloudOri -> points[i];
 //            // PointType pointProj = LaserCloudProj -> points[i];

 //            // float x1 = ctRoll * pointFrom.x - stRoll * pointFrom.y;
 //            // float y1 = stRoll * pointFrom.x + ctRoll* pointFrom.y;
 //            // float z1 = pointFrom.z;

 //            // float x2 = x1;
 //            // float y2 = ctPitch * y1 - stPitch * z1;
 //            // float z2 = stPitch * y1 + ctPitch* z1;

 //            // pointTo.x = ctYaw * x2 + stYaw * z2 + tInY;
 //            // pointTo.y = y2 + tInZ;
 //            // pointTo.z = -stYaw * x2 + ctYaw * z2 + tInX;

 //            // ROS_DEBUG("op to  %f %f %f ",pointTo.x, pointTo.y,pointTo.z);
 //            // ROS_DEBUG("op pro %f %f %f ",pointProj.x, pointProj.y,pointProj.z);
      

 //      Vector3d ori, proj;
 //      ori<< CloudKeyFramesOri[ind]->points[i].z , CloudKeyFramesOri[ind]->points[i].x , CloudKeyFramesOri[ind]->points[i].y;
      
 //      //ROS_DEBUG("ori111 %f %f %f  ",ori[0], ori[1], ori[2]);
 //      proj<< CloudKeyFramesProj[ind]->points[i].z, CloudKeyFramesProj[ind]->points[i].x , CloudKeyFramesProj[ind]->points[i].y;

 //      // ori<< pointFrom.z , pointFrom.x , pointFrom.y;
      
 //      // proj<< pointProj.z, pointProj.x , pointProj.y;

 //      double point[3] = {ori[0], ori[1], ori[2]};
      
 //      double q1[4] = {q.w(),q.x(),q.y(),q.z()};//!!!!!w x y z 

      
 //      ROS_DEBUG("q1111[4]  %f %f %f",q1[0],q1[1],q1[2]);
 //      double p[3];
 //      ceres::QuaternionRotatePoint( q1, point, p);
 //        p[0] += tInX;
 //        p[1] += tInY;
 //        p[2] += tInZ;
 //      cout << "ori2"<<p[0]<<" " <<p[1]<<" "<<p[2]<< endl;
 //      cout << "pro2"<<proj[0]<<" " << proj[1]<<" "<<proj[2]<<endl;


 //  }
 // // slideWindow();

}



void imuProcess(std::vector <sensor_msgs::ImuConstPtr> &IMU_Cur)
{
  for (auto &imu_msg : IMU_Cur){
    double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
    double imu_t = imu_msg->header.stamp.toSec();
    double lidar_t = timeLaserCloudOri + td;//img=img+td
    if (imu_t <= lidar_t)//imu<=img
    { 
      if (current_time < 0)
        current_time = imu_t;
        double dt = imu_t - current_time;//current: first_imu time
        ROS_ASSERT(dt >= 0);
        current_time = imu_t;
        dx = imu_msg->linear_acceleration.x;
        dy = imu_msg->linear_acceleration.y;
        dz = imu_msg->linear_acceleration.z;
        rx = imu_msg->angular_velocity.x;
        ry = imu_msg->angular_velocity.y;
        rz = imu_msg->angular_velocity.z;
        processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));//imu propogate 得位置 速度初始值 直到img时刻
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);

    }
  }
}

void processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0)//frame count increase  more than windowsize
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);//push back 包含了误差状态预积分
        //if(solver_flag != NON_LINEAR)
            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;         
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - G;//按前一帧
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];//前一帧与当前帧结合
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - G;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;//当前帧的姿态 位置 速度 
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;  
}



void optimizationProcess()
{
  if(frame_count == WINDOW_SIZE){
 
    solveOdometry();
    pubOptOdometry();
    slideWindow();
  }

 else
  frame_count++;
}


void solveOdometry()
{
  if(frame_count < WINDOW_SIZE)
    return;
  
  ROS_DEBUG("solveOdometry %d", frame_count);
  ceres::Problem problem;
  ceres::LossFunction *loss_function = new ceres::CauchyLoss(1);
  vector2double();

  for(int i=0; i< WINDOW_SIZE+1 ;i++)
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
      if(k < 3 && ind == WINDOW_SIZE-1){
      double point[3] = {ori[0], ori[1], ori[2]};
      double q[4]     = {para_Pose[ind][6],para_Pose[ind][3],para_Pose[ind][4],para_Pose[ind][5]};//!!!!!w x y z 
      double p[3];


      ceres::QuaternionRotatePoint( q, point, p);
        
        p[0] += para_Pose[ind][0];
        p[1] += para_Pose[ind][1];
        p[2] += para_Pose[ind][2];
      cout << "ori"<<p[0]<<" " <<p[1]<<" "<<p[2]<< endl;
      cout << "pro"<<proj[0]<<" " << proj[1]<<" "<<proj[2]<<endl;

      }
  

      // }  
      //ROS_DEBUG("ori x y  %f %f", ori[0],ori[1]);
      if(ori[0] > 50 ||  ori[1] > 50  || ori[2] > 50)
        continue;

      
      ceres::CostFunction* lidar_cost_function = ICPCostFunctions::PointToPointError_EigenQuaternion::Create(proj,ori);
      
      problem.AddResidualBlock(lidar_cost_function, loss_function, para_Pose[ind]);
      
      }
  }

   solveProblem(problem);
   double2vector();
}

void vector2double(){
  for (int i = 0; i <= WINDOW_SIZE ; i++)
    {
        para_Pose[i][0] = Ps[i][0];
        para_Pose[i][1] = Ps[i][1];
        para_Pose[i][2] = Ps[i][2];
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
    ROS_DEBUG("P-origin %f",Ps[WINDOW_SIZE][0]);

}


void double2vector()
{
    for (int i = 0; i <= WINDOW_SIZE ; i++)
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
    ROS_DEBUG("P-coupled %f",Ps[WINDOW_SIZE][0]);
}



void slideWindow()
{
  if(frame_count == WINDOW_SIZE)
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
    Ps[WINDOW_SIZE]  = Ps[WINDOW_SIZE - 1];
    Vs[WINDOW_SIZE]  = Vs[WINDOW_SIZE - 1];
    Rs[WINDOW_SIZE]  = Rs[WINDOW_SIZE - 1];
    Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
    Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];


    // delete pre_integrations[WINDOW_SIZE];
    // pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

    // dt_buf[WINDOW_SIZE].clear();
    // linear_acceleration_buf[WINDOW_SIZE].clear();
    // angular_velocity_buf[WINDOW_SIZE].clear();

  //}

    CloudKeyFramesOri.pop_front();
    CloudKeyFramesProj.pop_front();
    

  ROS_DEBUG("slideWindow frame_count%d", frame_count);
}
}


void pubOptOdometry(){
 
  //nav_msgs::Odometry odomCoupled;
  //odometry.header = lidarHeader;
  odomCoupled.header.stamp = ros::Time().fromSec(timeKeyPoses);
  odomCoupled.header.frame_id = "camera_init";
  odomCoupled.child_frame_id = "aft_coupled";
  Quaterniond tmp_Q;
  tmp_Q = Quaterniond(Rs[WINDOW_SIZE]);
  odomCoupled.pose.pose.position.x = Ps[WINDOW_SIZE].x();
  odomCoupled.pose.pose.position.y = Ps[WINDOW_SIZE].y();
  odomCoupled.pose.pose.position.z = Ps[WINDOW_SIZE].z();
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
  

  ROS_DEBUG("pubOdometry %d %f", frame_count, Ps[WINDOW_SIZE].x());

}

// std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloud2ConstPtr>>
// getMeasurements()
// {
//   std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloud2ConstPtr>> measurements;//imu的容器及一帧图片 图片时间在最后
//   while (true)
//     {
//       int sum_of_wait = 0;
//         if (imu_buf.empty() || ori_buf.empty())
//             return measurements;

//         if (!(imu_buf.back()->header.stamp.toSec() > ori_buf.front()->header.stamp.toSec() + td))// latest imu back<=img+td
//         {
//             //ROS_WARN("wait for imu, only should happen at the beginning");
//             sum_of_wait++;
//             return measurements;
//         }

//         if (!(imu_buf.front()->header.stamp.toSec() < ori_buf.front()->header.stamp.toSec() + td))//imu front>=img+t throw img
//         {
//             ROS_WARN("throw img, only should happen at the beginning");
//             ori_buf.pop();
//             continue;
//         }
//         sensor_msgs::PointCloud2ConstPtr lidar_ori = ori_buf.front();
//         ori_buf.pop();
//         //feature_buf.pop();

//         std::vector<sensor_msgs::ImuConstPtr> IMUs;
//         while (imu_buf.front()->header.stamp.toSec() < ori_buf.front()->header.stamp.toSec() + td)//imu front<img+t throw imu
//         {
//             IMUs.emplace_back(imu_buf.front());
//             imu_buf.pop();
//         }
//         IMUs.emplace_back(imu_buf.front());
//         if (IMUs.empty())
//             ROS_WARN("no imu between two image");
//         measurements.emplace_back(IMUs, LaserCloudOri);//最后一帧为IMU与img时间相等  前面为之间的IMU
//     }
//     return measurements;

// }


std::vector<sensor_msgs::ImuConstPtr> getMeasurements()
{
    
      std::vector<sensor_msgs::ImuConstPtr> IMUs;
      int sum_of_wait = 0;
        if (imu_buf.empty() )
            return IMUs;

        if (!(imu_buf.back()->header.stamp.toSec() > timeLaserCloudOri + td))// latest imu back<=img+td
        {
            //ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return IMUs;
        }

        if (!(imu_buf.front()->header.stamp.toSec() < timeLaserCloudOri + td))//imu front>=img+t throw img
        {
            ROS_WARN("throw img, only should happen at the beginning");
            return IMUs;           
        }
        //feature_buf.pop();
     

        
        while (imu_buf.front()->header.stamp.toSec() < timeLaserCloudOri + td)//imu front<img+t throw imu
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            ROS_WARN("no imu between two image");
        //最后一帧为IMU与img时间相等  前面为之间的IMU
    

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

    //std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloud2ConstPtr>> measurements;//vector imu , pointcloud的pair组成vector
    
   // std::vector <sensor_msgs::ImuConstPtr> IMU_Cur = getMeasurements();

    //imuProcess(IMU_Cur);
    

    currrentPoseProcess();

    

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