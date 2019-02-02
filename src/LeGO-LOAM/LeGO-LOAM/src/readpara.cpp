#include "readpara.h"
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <fstream>


double ex_roll;
double ex_pitch;
double ex_yaw;
Eigen::Matrix3d R_imutolidar;
Eigen::Quaterniond q_imutolidar;

double loss_lidar;
double loss_imu;
double s_thres;
bool imuAutodiff;
bool setBiasConst;
bool initial_enable;

bool smallProblem=false;


int num_threads;
double function_tolerance;

Eigen::Vector3d G = {0, 0, 9.794};

double td;
double PI = 3.1415926;

double ACC_N, ACC_W;
double GYR_N, GYR_W;

void readParameter(const char* filename)
{
  
    std::stringstream buffer;
    std::string line;
    std::string paramName;
    std::string paramValuestr;

    double paraValue = 0;
    int paraInt = 0;
    bool paraFlag = false;

    std::ifstream fin(filename);
    if (!fin.good())
    {
        std::string msg("parameters file not found");
        msg.append(filename);
        throw std::runtime_error(msg);
    }
    while (fin.good())
    {
        getline(fin,line);
        if(line[0] != '#')
        {
            buffer.clear();
            buffer << line;
            buffer >> paramName;

            if(paramName.compare("ex_roll") == 0)
            {
                buffer >>paraValue;
                ex_roll = paraValue *PI /180;
                cout<<"ex_roll: "<<paraValue<<endl;
            }
            else if(paramName.compare("ex_pitch") == 0)
            {
                buffer >>paraValue;
                ex_pitch = paraValue*PI /180;
                cout<<"ex_pitch: "<<paraValue<<endl;
            }
            else if(paramName.compare("ex_yaw") == 0)
            {
                buffer >>paraValue;
                ex_yaw = paraValue*PI /180;
                cout<<"ex_yaw: "<<paraValue<<endl;
            }
             else if(paramName.compare("td") == 0)
            {
                buffer >>paraValue;
                td = paraValue;
                cout<<"td: "<<td<<endl;
            }
              else if(paramName.compare("G") == 0)
            {
                buffer >>paraValue;
                G[2] = paraValue;
                cout<<"G: "<<G[2]<<endl;
            }
              else if(paramName.compare("loss_lidar") == 0)
            {
                buffer >>paraValue;
                loss_lidar = paraValue;
                cout<<"loss_lidar: "<<loss_lidar<<endl;
            }
              else if(paramName.compare("loss_imu") == 0)
            {
                buffer >>paraValue;
                loss_imu = paraValue;
                cout<<"loss_imu: "<<loss_imu<<endl;
            }
             else if(paramName.compare("s_thres") == 0)
            {
                buffer >>paraValue;
                s_thres = paraValue;
                cout<<"s_thres: "<<s_thres<<endl;
            }
              else if(paramName.compare("smallProblem") == 0)
            {
                buffer >>paraFlag;
                smallProblem = paraFlag;
                cout<<"smallProblem: "<<smallProblem<<endl;
            }
             else if(paramName.compare("imuAutodiff") == 0)
            {
                buffer >>paraFlag;
                imuAutodiff = paraFlag;
                cout<<"imuAutodiff: "<<imuAutodiff<<endl;
            }
            else if(paramName.compare("initial_enable") == 0)
            {
                buffer >>paraFlag;
                initial_enable = paraFlag;
                cout<<"initial_enable: "<<initial_enable<<endl;
            }
               else if(paramName.compare("num_threads") == 0)
            {
                buffer >>paraInt;
                num_threads = paraInt;
                cout<<"num_threads: "<<num_threads<<endl;
            }
                 else if(paramName.compare("function_tolerance") == 0)
            {
                buffer >>paraValue;
                function_tolerance = paraValue;
                cout<<"function_tolerance: "<<function_tolerance<<endl;
            }
               else if(paramName.compare("ACC_N") == 0)
            {
                buffer >>paraValue;
                ACC_N = paraValue;
                cout<<"ACC_N: "<<ACC_N<<endl;
            }
               else if(paramName.compare("ACC_W") == 0)
            {
                buffer >>paraValue;
                ACC_W = paraValue;
                cout<<"ACC_W: "<<ACC_W<<endl;
            }
               else if(paramName.compare("GYR_N") == 0)
            {
                buffer >>paraValue;
                GYR_N = paraValue;
                cout<<"GYR_N: "<<GYR_N<<endl;
            }
               else if(paramName.compare("GYR_W") == 0)
            {
                buffer >>paraValue;
                GYR_W = paraValue;
                cout<<"GYR_W: "<<GYR_W<<endl;
            }
               else if(paramName.compare("setBiasConst") == 0)
            {
                buffer >>paraFlag;
                setBiasConst = paraFlag;
                cout<<"setBiasConst: "<<setBiasConst<<endl;
            }
            else
                throw std::runtime_error(std::string("unknown parameter"));
        }

    }

    fin.close();
}