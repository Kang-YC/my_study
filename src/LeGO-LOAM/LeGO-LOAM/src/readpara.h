#pragma once

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace Eigen;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;
extern double LIDAR_N;


extern double ex_roll;
extern double ex_pitch;
extern double ex_yaw;
extern Eigen::Matrix3d R_imutolidar;
extern Eigen::Quaterniond q_imutolidar;

extern double loss_lidar;
extern double loss_imu;
extern double s_thres;
extern bool imuAutodiff;
extern bool setBiasConst;
extern bool initial_enable;

extern bool smallProblem;


extern int num_threads;
extern double function_tolerance;

extern Eigen::Vector3d G ;

extern double td;
extern double scale;


void readParameter(const char* filename);

