#ifndef ICPCERES
#define ICPCERES

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

//#include "frame.h"

#include <ceres/local_parameterization.h>
#include <ceres/autodiff_local_parameterization.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/types.h>

#include <ceres/rotation.h>
#include "eigen_quaternion.h"
//#include "sophus_se3.h"
#include "../src/factor/integration_base.h"


using namespace Eigen;
using namespace std;

namespace ICP_Ceres {

template <typename T>
ceres::MatrixAdapter<T, 1, 4> ColumnMajorAdapter4x3(T* pointer) {
  return ceres::MatrixAdapter<T, 1, 4>(pointer);
}

    //pairwise
    Isometry3d pointToPoint_EigenQuaternion(vector<Vector3d> &src,vector<Vector3d> &dst);
    Isometry3d pointToPoint_CeresAngleAxis(vector<Vector3d> &src,vector<Vector3d> &dst);
    //Isometry3d pointToPoint_SophusSE3(vector<Vector3d> &src,vector<Vector3d> &dst,bool automaticDiffLocalParam=true);

    Isometry3d pointToPlane_EigenQuaternion(vector<Vector3d> &src,vector<Vector3d> &dst,vector<Vector3d> &nor);
    Isometry3d pointToPlane_CeresAngleAxis(vector<Vector3d> &src,vector<Vector3d> &dst,vector<Vector3d> &nor);
    //Isometry3d pointToPlane_SophusSE3(vector<Vector3d> &src,vector<Vector3d> &dst,vector<Vector3d> &nor,bool automaticDiffLocalParam=true);


    /*//multiview
    void ceresOptimizer(std::vector< std::shared_ptr<Frame> >& frames, bool pointToPlane, bool robust);
    void ceresOptimizer_ceresAngleAxis(vector< std::shared_ptr<Frame> >& frames, bool pointToPlane, bool robust);
    void ceresOptimizer_sophusSE3(vector< std::shared_ptr<Frame> >& frames, bool pointToPlane, bool robust, bool automaticDiffLocalParam=true);
*/

}

namespace ICPCostFunctions{

struct PointToPointErrorGlobal{

    const Eigen::Vector3d p_dst;
    const Eigen::Vector3d p_src;

    PointToPointErrorGlobal(const Eigen::Vector3d &dst, const Eigen::Vector3d &src) :
        p_dst(dst), p_src(src)
    {

    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d& dst, const Eigen::Vector3d& src) {
        return (new ceres::AutoDiffCostFunction<PointToPointErrorGlobal, 3, 4, 3, 4, 3>(new PointToPointErrorGlobal(dst, src)));
    }

    template <typename T>
    bool operator()(const T* const camera_rotation, const T* const camera_translation, const T* const camera_rotation_dst, const T* const camera_translation_dst, T* residuals) const {

        // Make sure the Eigen::Vector world point is using the ceres::Jet type as it's Scalar type
        Eigen::Matrix<T,3,1> src; src << T(p_src[0]), T(p_src[1]), T(p_src[2]);
        Eigen::Matrix<T,3,1> dst; dst << T(p_dst[0]), T(p_dst[1]), T(p_dst[2]);

        // Map the T* array to an Eigen Quaternion object (with appropriate Scalar type)
        Eigen::Quaternion<T> q = Eigen::Map<const Eigen::Quaternion<T> >(camera_rotation);
        Eigen::Quaternion<T> qDst = Eigen::Map<const Eigen::Quaternion<T> >(camera_rotation_dst);

        // Map T* to Eigen Vector3 with correct Scalar type
        Eigen::Matrix<T,3,1> t = Eigen::Map<const Eigen::Matrix<T,3,1> >(camera_translation);
        Eigen::Matrix<T,3,1> tDst = Eigen::Map<const Eigen::Matrix<T,3,1> >(camera_translation_dst);

        // Rotate the point using Eigen rotations
        Eigen::Matrix<T,3,1> p = q * src;
        p += t;
        Eigen::Matrix<T,3,1> p2 = qDst * dst;
        p2 += tDst;

        // The error is the difference between the predicted and observed position.
        residuals[0] = p[0] - p2[0];
        residuals[1] = p[1] - p2[1];
        residuals[2] = p[2] - p2[2];

        return true;
    }
};

struct PointToPlaneErrorGlobal{

    const Eigen::Vector3d& p_dst;
    const Eigen::Vector3d& p_src;
    const Eigen::Vector3d& p_nor;


    PointToPlaneErrorGlobal(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const Eigen::Vector3d& nor) :
    p_dst(dst), p_src(src), p_nor(nor)
    {
//        cout<<nor.dot(nor)<<endl;
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const Eigen::Vector3d& nor) {
        return (new ceres::AutoDiffCostFunction<PointToPlaneErrorGlobal, 1, 4, 3, 4, 3>(new PointToPlaneErrorGlobal(dst, src, nor)));
    }

    template <typename T>
    bool operator()(const T* const camera_rotation, const T* const camera_translation, const T* const camera_rotation_dst, const T* const camera_translation_dst, T* residuals) const {

        // Make sure the Eigen::Vector world point is using the ceres::Jet type as it's Scalar type
        Eigen::Matrix<T,3,1> src; src << T(p_src[0]), T(p_src[1]), T(p_src[2]);
        Eigen::Matrix<T,3,1> dst; dst << T(p_dst[0]), T(p_dst[1]), T(p_dst[2]);
        Eigen::Matrix<T,3,1> nor; nor << T(p_nor[0]), T(p_nor[1]), T(p_nor[2]);

        // Map the T* array to an Eigen Quaternion object (with appropriate Scalar type)
        Eigen::Quaternion<T> q = Eigen::Map<const Eigen::Quaternion<T> >(camera_rotation);
        Eigen::Quaternion<T> qDst = Eigen::Map<const Eigen::Quaternion<T> >(camera_rotation_dst);

        // Map T* to Eigen Vector3 with correct Scalar type
        Eigen::Matrix<T,3,1> t = Eigen::Map<const Eigen::Matrix<T,3,1> >(camera_translation);
        Eigen::Matrix<T,3,1> tDst = Eigen::Map<const Eigen::Matrix<T,3,1> >(camera_translation_dst);

        // Rotate the point using Eigen rotations
        Eigen::Matrix<T,3,1> p = q * src;
        p += t;
        Eigen::Matrix<T,3,1> p2 = qDst.toRotationMatrix() * dst;
        p2 += tDst;
        Eigen::Matrix<T,3,1> n2 = qDst.toRotationMatrix() * nor; //no translation on normal

        // The error is the difference between the predicted and observed position projected onto normal
        residuals[0] = (p - p2).dot(n2);//the projection according to the normal

        return true;
    }
};

struct PointToPointErrorGlobal_CeresAngleAxis{

    const Eigen::Vector3d& p_dst;
    const Eigen::Vector3d& p_src;

    PointToPointErrorGlobal_CeresAngleAxis(const Eigen::Vector3d &dst, const Eigen::Vector3d &src) :
        p_dst(dst), p_src(src)
    {
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d &observed, const Eigen::Vector3d &worldPoint) {
        return (new ceres::AutoDiffCostFunction<PointToPointErrorGlobal_CeresAngleAxis, 3, 6, 6>(new PointToPointErrorGlobal_CeresAngleAxis(observed, worldPoint)));
    }

    template <typename T>
    bool operator()(const T* const camera, const T* const camera2, T* residuals) const {

        T p[3] = {T(p_src[0]), T(p_src[1]), T(p_src[2])};
        ceres::AngleAxisRotatePoint(camera,p,p);

        T p2[3] = {T(p_dst[0]), T(p_dst[1]), T(p_dst[2])};
        ceres::AngleAxisRotatePoint(camera2,p2,p2);

        // camera[3,4,5] are the translation.
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        p2[0] += camera2[3];
        p2[1] += camera2[4];
        p2[2] += camera2[5];

        // The error is the difference between the predicted and observed position.
        residuals[0] = p[0] - p2[0];
        residuals[1] = p[1] - p2[1];
        residuals[2] = p[2] - p2[2];

        return true;
    }
};

struct PointToPlaneErrorGlobal_CeresAngleAxis{

    const Eigen::Vector3d& p_dst;
    const Eigen::Vector3d& p_src;
    const Eigen::Vector3d& p_nor;


    PointToPlaneErrorGlobal_CeresAngleAxis(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const Eigen::Vector3d& nor) :
    p_dst(dst), p_src(src), p_nor(nor)
    {
//        cout<<nor.dot(nor)<<endl;
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const Eigen::Vector3d& nor) {
        return (new ceres::AutoDiffCostFunction<PointToPlaneErrorGlobal_CeresAngleAxis, 1, 6, 6>(new PointToPlaneErrorGlobal_CeresAngleAxis(dst, src, nor)));
    }

    template <typename T>
    bool operator()(const T* const cam1, const T* const cam2, T* residuals) const {

        T p1[3] = {T(p_src[0]), T(p_src[1]), T(p_src[2])};
        ceres::AngleAxisRotatePoint(cam1,p1,p1);

        T p2[3] = {T(p_dst[0]), T(p_dst[1]), T(p_dst[2])};
        ceres::AngleAxisRotatePoint(cam2,p2,p2);

        T nor[3] = {T(p_nor[0]), T(p_nor[1]), T(p_nor[2])};
        ceres::AngleAxisRotatePoint(cam2,nor,nor);

        // camera[3,4,5] are the translation.
        p1[0] += cam1[3];
        p1[1] += cam1[4];
        p1[2] += cam1[5];

        p2[0] += cam2[3];
        p2[1] += cam2[4];
        p2[2] += cam2[5];

        //no translation on normal

        // The error is the difference between the predicted and observed position projected onto normal
        residuals[0] = (p1[0] - p2[0]) * nor[0] + \
                       (p1[1] - p2[1]) * nor[1] + \
                       (p1[2] - p2[2]) * nor[2];


        return true;
    }
};
/*
struct PointToPointErrorGlobal_SophusSE3{

    const Eigen::Vector3d p_dst;
    const Eigen::Vector3d p_src;

    PointToPointErrorGlobal_SophusSE3(const Eigen::Vector3d &dst, const Eigen::Vector3d &src) :
        p_dst(dst), p_src(src)
    {

    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d& dst, const Eigen::Vector3d& src) {
        return (new ceres::AutoDiffCostFunction<PointToPointErrorGlobal_SophusSE3, 3, Sophus::SE3d::num_parameters, Sophus::SE3d::num_parameters>(new PointToPointErrorGlobal_SophusSE3(dst, src)));
    }

    template <typename T>
    bool operator()(const T* const cam1, const T* const cam2, T* residuals) const {

        // Make sure the Eigen::Vector world point is using the ceres::Jet type as it's Scalar type
        Eigen::Matrix<T,3,1> src; src << T(p_src[0]), T(p_src[1]), T(p_src[2]);
        Eigen::Matrix<T,3,1> dst; dst << T(p_dst[0]), T(p_dst[1]), T(p_dst[2]);

        // Map the T* array to an Sophus SE3 object (with appropriate Scalar type)
        Sophus::SE3Group<T> q = Eigen::Map< const Sophus::SE3Group<T> >(cam1);
        Sophus::SE3Group<T> q2 = Eigen::Map< const Sophus::SE3Group<T> >(cam2);

        // Rotate the point using Eigen rotations
        Eigen::Matrix<T,3,1> p = q.unit_quaternion() * src + q.translation();
        Eigen::Matrix<T,3,1> p2 = q2.unit_quaternion() * dst + q2.translation();

        // The error is the difference between the predicted and observed position.
        residuals[0] = p[0] - p2[0];
        residuals[1] = p[1] - p2[1];
        residuals[2] = p[2] - p2[2];

        return true;
    }
};

struct PointToPlaneErrorGlobal_SophusSE3{

    const Eigen::Vector3d& p_dst;
    const Eigen::Vector3d& p_src;
    const Eigen::Vector3d& p_nor;


    PointToPlaneErrorGlobal_SophusSE3(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const Eigen::Vector3d& nor) :
    p_dst(dst), p_src(src), p_nor(nor)
    {
//        cout<<nor.dot(nor)<<endl;
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const Eigen::Vector3d& nor) {
        return (new ceres::AutoDiffCostFunction<PointToPlaneErrorGlobal_SophusSE3, 1, Sophus::SE3d::num_parameters, Sophus::SE3d::num_parameters>(new PointToPlaneErrorGlobal_SophusSE3(dst, src, nor)));
    }

    template <typename T>
    bool operator()(const T* const cam1, const T* const cam2, T* residuals) const {

        // Make sure the Eigen::Vector world point is using the ceres::Jet type as it's Scalar type
        Eigen::Matrix<T,3,1> src; src << T(p_src[0]), T(p_src[1]), T(p_src[2]);
        Eigen::Matrix<T,3,1> dst; dst << T(p_dst[0]), T(p_dst[1]), T(p_dst[2]);
        Eigen::Matrix<T,3,1> nor; nor << T(p_nor[0]), T(p_nor[1]), T(p_nor[2]);

        // Map the T* array to an Sophus SE3 object (with appropriate Scalar type)
        Sophus::SE3Group<T> q = Eigen::Map< const Sophus::SE3Group<T> >(cam1);
        Sophus::SE3Group<T> q2 = Eigen::Map< const Sophus::SE3Group<T> >(cam2);

        // Rotate the point using Eigen rotations
        Eigen::Matrix<T,3,1> p = q.unit_quaternion() * src + q.translation();
        Eigen::Matrix<T,3,1> p2 = q2.unit_quaternion() * dst + q2.translation();
        Eigen::Matrix<T,3,1> n2 = q2.unit_quaternion() * nor; //no translation on normal

        // The error is the difference between the predicted and observed position projected onto normal
        residuals[0] = (p - p2).dot(n2);

        return true;
    }
};
*/


struct PointToPointError_CeresAngleAxis{

    const Eigen::Vector3d& p_dst;
    const Eigen::Vector3d& p_src;

    PointToPointError_CeresAngleAxis(const Eigen::Vector3d &dst, const Eigen::Vector3d &src) :
        p_dst(dst), p_src(src)
    {
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d &observed, const Eigen::Vector3d &worldPoint) {
        return (new ceres::AutoDiffCostFunction<PointToPointError_CeresAngleAxis, 3, 6>(new PointToPointError_CeresAngleAxis(observed, worldPoint)));
    }

    template <typename T>
    bool operator()(const T* const camera, T* residuals) const {

//            Eigen::Matrix<T,3,1> point;
//            point << T(p_src[0]), T(p_src[1]), T(p_src[2]);

        T p[3] = {T(p_src[0]), T(p_src[1]), T(p_src[2])};
        ceres::AngleAxisRotatePoint(camera,p,p);

        // camera[3,4,5] are the translation.
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        // The error is the difference between the predicted and observed position.
        residuals[0] = p[0] - T(p_dst[0]);
        residuals[1] = p[1] - T(p_dst[1]);
        residuals[2] = p[2] - T(p_dst[2]);

        return true;
    }
};

// struct PointToPointError_EigenQuaternion{
//     const Eigen::Vector3d& p_dst;
//     const Eigen::Vector3d& p_src;

//     PointToPointError_EigenQuaternion(const Eigen::Vector3d &dst, const Eigen::Vector3d &src) :
//         p_dst(dst), p_src(src)
//     {
//     }

//     // Factory to hide the construction of the CostFunction object from the client code.
//     static ceres::CostFunction* Create(const Eigen::Vector3d &observed, const Eigen::Vector3d &worldPoint) {
//         return (new ceres::AutoDiffCostFunction<PointToPointError_EigenQuaternion, 3, 4, 3>(new PointToPointError_EigenQuaternion(observed, worldPoint)));
//     }

//     template <typename T>
//     bool operator()(const T* const camera_rotation, const T* const camera_translation, T* residuals) const {

//         //ceres::AngleAxisRotatePoint

//         // Make sure the Eigen::Vector world point is using the ceres::Jet type as it's Scalar type
//         Eigen::Matrix<T,3,1> point; point << T(p_src[0]), T(p_src[1]), T(p_src[2]);

//         // Map the T* array to an Eigen Quaternion object (with appropriate Scalar type)
//         Eigen::Quaternion<T> q = Eigen::Map<const Eigen::Quaternion<T> >(camera_rotation);

//         // Rotate the point using Eigen rotations
//         Eigen::Matrix<T,3,1> p = q * point;

//         // Map T* to Eigen Vector3 with correct Scalar type
//         Eigen::Matrix<T,3,1> t = Eigen::Map<const Eigen::Matrix<T,3,1> >(camera_translation);
//         p += t;

//         // The error is the difference between the predicted and observed position.
//         residuals[0] = p[0] - T(p_dst[0]);
//         residuals[1] = p[1] - T(p_dst[1]);
//         residuals[2] = p[2] - T(p_dst[2]);

//         return true;
//     }
// };
// 
// 
struct PointToPointError_EigenQuaternion{
    const Eigen::Vector3d& p_dst;
    const Eigen::Vector3d& p_src;
    double LIDAR_N =1000;

    PointToPointError_EigenQuaternion(const Eigen::Vector3d &dst, const Eigen::Vector3d &src) :
        p_dst(dst), p_src(src)
    {
    }
    //dist in world    src in body

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d &observed, const Eigen::Vector3d &worldPoint) {
        return (new ceres::AutoDiffCostFunction<PointToPointError_EigenQuaternion, 3, 7>(new PointToPointError_EigenQuaternion(observed, worldPoint)));
    }

    template <typename T>
    bool operator()(const T* const para_Pose,  T* residuals) const {

        //ceres::AngleAxisRotatePoint

        // Make sure the Eigen::Vector world point is using the ceres::Jet type as it's Scalar type
        //Eigen::Matrix<T,3,1> point; point << T(p_src[0]), T(p_src[1]), T(p_src[2]);
        //Eigen::Map<Eigen::Matrix<T, 3, 1>> residual(residuals);

        T point[3] = {T(p_src[0]), T(p_src[1]), T(p_src[2])};
        T q[4] = {para_Pose[6],para_Pose[3],para_Pose[4],para_Pose[5]};//ORDER!!!!!!!!!!!!!!
        T p[3];

        ceres::QuaternionRotatePoint( q, point, p);
      
        p[0] += para_Pose[0];
        p[1] += para_Pose[1];
        p[2] += para_Pose[2];

        // cout <<"ori" <<p[0] <<p[1]<<p[2]<< endl;
        // cout<< p_dst<<endl;

 
        residuals[0] = LIDAR_N*(p[0] - T(p_dst[0]));
        residuals[1] = LIDAR_N*(p[1] - T(p_dst[1]));
        residuals[2] = LIDAR_N*(p[2] - T(p_dst[2]));


        // Eigen::Matrix<T, 3, 3> sqrt_info_lidar = LIDAR_N * Eigen::Matrix3d::Identity();

        // residual = sqrt_info_lidar *residual ;


 
        // residuals[0] = 0.2*(p[0] - T(p_dst[0]));
        // residuals[1] = 0.1*(p[1] - T(p_dst[1]));
        // residuals[2] = 0.1*(p[2] - T(p_dst[2]));

        return true;
    }
};

//use double

// struct IMUFactor{
//     const IntegrationBase* pre_integration;

//     IMUFactor(const IntegrationBase* _pre_integration) :pre_integration(_pre_integration)     
//     {
//     }

//     // Factory to hide the construction of the CostFunction object from the client code.
//     static ceres::CostFunction* Create(const IntegrationBase* pre_integration) {
//         return (new ceres::AutoDiffCostFunction<IMUFactor, 15, 7, 9, 7, 9>(new IMUFactor(pre_integration)));
//     }

//     template <typename T>
//     bool operator()(const T* const para_Pose_i, const T* const para_Speedbias_i, const T* const para_Pose_j ,const T* const para_Speedbias_j, T* residuals) const 
//     {

//         Eigen::Map<Eigen::Matrix<T, 15, 1>> residual(residuals);


//         Eigen::Vector3d Pi(para_Pose_i[0], para_Pose_i[1], para_Pose_i[2]);
//         Eigen::Quaterniond Qi(para_Pose_i[6], para_Pose_i[3], para_Pose_i[4], para_Pose_i[5]);

//         Eigen::Vector3d Vi(para_Speedbias_i[0], para_Speedbias_i[1], para_Speedbias_i[2]);
//         Eigen::Vector3d Bai(para_Speedbias_i[3], para_Speedbias_i[4], para_Speedbias_i[5]);
//         Eigen::Vector3d Bgi(para_Speedbias_i[6], para_Speedbias_i[7], para_Speedbias_i[8]);

//         Eigen::Vector3d Pj(para_Pose_j[0], para_Pose_j[1], para_Pose_j[2]);
//         Eigen::Quaterniond  Qj(para_Pose_j[6], para_Pose_j[3], para_Pose_j[4], para_Pose_j[5]);
//         Eigen::Quaterniond  Qj_inverse = Qi.inverse();

//         Eigen::Vector3d Vj(para_Speedbias_j[0], para_Speedbias_j[1], para_Speedbias_j[2]);
//         Eigen::Vector3d Baj(para_Speedbias_j[3], para_Speedbias_j[4], para_Speedbias_j[5]);
//         Eigen::Vector3d Bgj(para_Speedbias_j[3], para_Speedbias_j[4], para_Speedbias_j[5]);


        
//         // Eigen::Matrix<T,3,1> Pi(para_Pose_i[0], para_Pose_i[1], para_Pose_i[2]);
//         // Eigen::Quaternion<T> Qi(para_Pose_i[6], para_Pose_i[3], para_Pose_i[4], para_Pose_i[5]);

//         // Eigen::Matrix<T,3,1> Vi(para_Speedbias_i[0], para_Speedbias_i[1], para_Speedbias_i[2]);
//         // Eigen::Matrix<T,3,1> Bai(para_Speedbias_i[3], para_Speedbias_i[4], para_Speedbias_i[5]);
//         // Eigen::Matrix<T,3,1> Bgi(para_Speedbias_i[6], para_Speedbias_i[7], para_Speedbias_i[8]);

//         // Eigen::Matrix<T,3,1> Pj(para_Pose_j[0], para_Pose_j[1], para_Pose_j[2]);
//         // Eigen::Quaternion<T>  Qj(para_Pose_j[6], para_Pose_j[3], para_Pose_j[4], para_Pose_j[5]);
//         // Eigen::Quaternion<T>  Qj_inverse = Qi.inverse();

//         // Eigen::Matrix<T,3,1> Vj(para_Speedbias_j[0], para_Speedbias_j[1], para_Speedbias_j[2]);
//         // Eigen::Matrix<T,3,1> Baj(para_Speedbias_j[3], para_Speedbias_j[4], para_Speedbias_j[5]);
//         // Eigen::Matrix<T,3,1> Bgj(para_Speedbias_j[3], para_Speedbias_j[4], para_Speedbias_j[5]);

        
//         // Eigen::Matrix3d dp_dba = pre_integration->jacobian.block<3, 3>(O_P, O_BA);
//         // Eigen::Matrix3d dp_dbg = pre_integration->jacobian.block<3, 3>(O_P, O_BG);
//         // Eigen::Matrix3d dq_dbg = pre_integration->jacobian.block<3, 3>(O_R, O_BG);
//         // Eigen::Matrix3d dv_dba = pre_integration->jacobian.block<3, 3>(O_V, O_BA);
//         // Eigen::Matrix3d dv_dbg = pre_integration->jacobian.block<3, 3>(O_V, O_BG);
//         // 
//       //   Eigen::Matrix<T,3,3> dp_dba = pre_integration->jacobian.block<3, 3>(O_P, O_BA).cast<T>();
//       //   Eigen::Matrix<T,3,3> dp_dbg = pre_integration->jacobian.block<3, 3>(O_P, O_BG).cast<T>();
//       //   Eigen::Matrix<T,3,3> dq_dbg = pre_integration->jacobian.block<3, 3>(O_R, O_BG).cast<T>();
//       //   Eigen::Matrix<T,3,3> dv_dba = pre_integration->jacobian.block<3, 3>(O_V, O_BA).cast<T>();
//       //   Eigen::Matrix<T,3,3> dv_dbg = pre_integration->jacobian.block<3, 3>(O_V, O_BG).cast<T>();//scale convet


//       // //  Eigen::Matrix<T,3,3> dp_dbg1 = dp_dba.cast<T>();

//       //   // // Eigen::Matrix<T,3,3> dp_dbg = pre_integration->jacobian.block<3, 3>(O_P, O_BG);
//       //   // // Eigen::Matrix<T,3,3> dq_dbg = pre_integration->jacobian.block<3, 3>(O_R, O_BG);
//       //   // // Eigen::Matrix<T,3,3> dv_dba = pre_integration->jacobian.block<3, 3>(O_V, O_BA);
//       //   // // Eigen::Matrix<T,3,3> dv_dbg = pre_integration->jacobian.block<3, 3>(O_V, O_BG);
//       //   // // 
//       //   // // 
//       //  // Eigen::Vector3d dba (static_cast<double>(Bai[0])-pre_integration->linearized_ba[0],static_cast<double>(Bai[1])-pre_integration->linearized_ba[1],static_const<double>(Bai[2])-pre_integration->linearized_ba[2]);
//       //   //Eigen::Vector3d dbg = double(Bgi) - pre_integration->linearized_bg;

//       //   // Eigen::Matrix<T,3,1> dba(Bai[0]-T(pre_integration->linearized_ba[0]),Bai[1]-T(pre_integration->linearized_ba[1]),Bai[2]-T(pre_integration->linearized_ba[2]));
//       //   // Eigen::Matrix<T,3,1> dbg(Bai[0]-T(pre_integration->linearized_bg[0]),Bai[1]-T(pre_integration->linearized_bg[1]),Bai[2]-T(pre_integration->linearized_bg[2]));


//       //   Eigen::Matrix<T,3,1> dba = Bai - pre_integration->linearized_ba.cast<T>();
//       //   Eigen::Matrix<T,3,1> dbg = Bgi - pre_integration->linearized_bg.cast<T>();

        
//       //   Eigen::Matrix<T,3,1> theta  = dq_dbg * dbg;
//       //   //Eigen::Vector3d theta  = dq_dbg * dbg;
//       //   T delta_theta[4] = {T(1) , T(theta[0]),T(theta[1]),T(theta[2])};
//       //   T delta_q1[4] = {T(pre_integration->delta_q.w()),T(pre_integration->delta_q.x()),T(pre_integration->delta_q.y()),T(pre_integration->delta_q.z())};
//       //   T corrected_delta_q[4] ;
//       //   ceres::QuaternionProduct( delta_q1, delta_theta, corrected_delta_q);

//       //   // // //Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * dbg);
//       //   // Eigen::Vector3d corrected_delta_v = pre_integration->delta_v + dv_dba * dba + dv_dbg * dbg;
//       //   // Eigen::Vector3d corrected_delta_p = pre_integration->delta_p + dp_dba * dba + dp_dbg * dbg;

//       //   Eigen::Matrix<T,3,1> corrected_delta_v = pre_integration->delta_v.cast<T>() + dv_dba * dba + dv_dbg * dbg;
//       //   Eigen::Matrix<T,3,1> corrected_delta_p = pre_integration->delta_p.cast<T>() + dp_dba * dba + dp_dbg * dbg;

//       //   // cout <<"corrected_delta_v" <<corrected_delta_v[0] <<corrected_delta_v[1]<<corrected_delta_v[2]<< endl;
//       //   // cout <<"corrected_delta_q" <<corrected_delta_q[0] <<corrected_delta_q[1]<<corrected_delta_q[2]<< endl;
//       //   // T t = T(pre_integration->sum_dt);

//       //   Eigen::Vector3d temp_P =0.5 * G * pre_integration->sum_dt ;

//       //    // Eigen::Matrix<T,3,1> temp_P =(0.5 * G.cast<T>() * t * t + Pj - Pi - Vi * t) - corrected_delta_p;
        
        

        


//         //residual.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * G * T(pre_integration->sum_dt) * T(pre_integration->sum_dt) + Pj - Pi - Vi * T(pre_integration->sum_dt)) - corrected_delta_p;
//         // residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
//         // residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_delta_v;
//         // residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
//         // residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;





//         // Eigen::Vector3d dba = Bai - linearized_ba;
//         // Eigen::Vector3d dbg = Bgi - linearized_bg;

    
//         // ceres::QuaternionRotatePoint( q, point, p);
    

//         return true;
//     }
// // };



// use template T
struct IMUFactor{
    const IntegrationBase* pre_integration;

    IMUFactor(const IntegrationBase* _pre_integration) :pre_integration(_pre_integration)     
    {
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const IntegrationBase* pre_integration) {
        return (new ceres::AutoDiffCostFunction<IMUFactor, 15, 7, 9, 7, 9>(new IMUFactor(pre_integration)));
    }

    template <typename T>
    bool operator()(const T* const para_Pose_i, const T* const para_Speedbias_i, const T* const para_Pose_j ,const T* const para_Speedbias_j, T* residuals) const 
    {
        Eigen::Map<Eigen::Matrix<T, 15, 1>> residual(residuals);

       // Eigen::Map<Eigen::Matrix<T, 15, 1>> residual(residuals);


        // Eigen::Vector3d Pi(para_Pose_i[0], para_Pose_i[1], para_Pose_i[2]);
        // Eigen::Quaterniond Qi(para_Pose_i[6], para_Pose_i[3], para_Pose_i[4], para_Pose_i[5]);

        // Eigen::Vector3d Vi(para_Speedbias_i[0], para_Speedbias_i[1], para_Speedbias_i[2]);
        // Eigen::Vector3d Bai(para_Speedbias_i[3], para_Speedbias_i[4], para_Speedbias_i[5]);
        // Eigen::Vector3d Bgi(para_Speedbias_i[6], para_Speedbias_i[7], para_Speedbias_i[8]);

        // Eigen::Vector3d Pj(para_Pose_j[0], para_Pose_j[1], para_Pose_j[2]);
        // Eigen::Quaterniond  Qj(para_Pose_j[6], para_Pose_j[3], para_Pose_j[4], para_Pose_j[5]);
        // Eigen::Quaterniond  Qj_inverse = Qi.inverse();

        // Eigen::Vector3d Vj(para_Speedbias_j[0], para_Speedbias_j[1], para_Speedbias_j[2]);
        // Eigen::Vector3d Baj(para_Speedbias_j[3], para_Speedbias_j[4], para_Speedbias_j[5]);
        // Eigen::Vector3d Bgj(para_Speedbias_j[3], para_Speedbias_j[4], para_Speedbias_j[5]);


        
        Eigen::Matrix<T,3,1> Pi(para_Pose_i[0], para_Pose_i[1], para_Pose_i[2]);
        Eigen::Quaternion<T> Qi(para_Pose_i[6], para_Pose_i[3], para_Pose_i[4], para_Pose_i[5]);
        Eigen::Quaternion<T> Qi_inverse = Qi.inverse();
        //cout<<"Pi"<<Pi<<endl;

        Eigen::Matrix<T,3,1> Vi(para_Speedbias_i[0], para_Speedbias_i[1], para_Speedbias_i[2]);
        Eigen::Matrix<T,3,1> Bai(para_Speedbias_i[3], para_Speedbias_i[4], para_Speedbias_i[5]);
        Eigen::Matrix<T,3,1> Bgi(para_Speedbias_i[6], para_Speedbias_i[7], para_Speedbias_i[8]);

        Eigen::Matrix<T,3,1> Pj(para_Pose_j[0], para_Pose_j[1], para_Pose_j[2]);
        Eigen::Quaternion<T>  Qj(para_Pose_j[6], para_Pose_j[3], para_Pose_j[4], para_Pose_j[5]);
        Eigen::Quaternion<T>  Qj_inverse = Qj.inverse();

        Eigen::Matrix<T,3,1> Vj(para_Speedbias_j[0], para_Speedbias_j[1], para_Speedbias_j[2]);
        Eigen::Matrix<T,3,1> Baj(para_Speedbias_j[3], para_Speedbias_j[4], para_Speedbias_j[5]);
        Eigen::Matrix<T,3,1> Bgj(para_Speedbias_j[6], para_Speedbias_j[7], para_Speedbias_j[8]);

        
        // Eigen::Matrix3d dp_dba = pre_integration->jacobian.block<3, 3>(O_P, O_BA);
        // Eigen::Matrix3d dp_dbg = pre_integration->jacobian.block<3, 3>(O_P, O_BG);
        // Eigen::Matrix3d dq_dbg = pre_integration->jacobian.block<3, 3>(O_R, O_BG);
        // Eigen::Matrix3d dv_dba = pre_integration->jacobian.block<3, 3>(O_V, O_BA);
        // Eigen::Matrix3d dv_dbg = pre_integration->jacobian.block<3, 3>(O_V, O_BG);
        // 
        Eigen::Matrix<T,3,3> dp_dba = pre_integration->jacobian.block<3, 3>(O_P, O_BA).cast<T>();
        Eigen::Matrix<T,3,3> dp_dbg = pre_integration->jacobian.block<3, 3>(O_P, O_BG).cast<T>();
        Eigen::Matrix<T,3,3> dq_dbg = pre_integration->jacobian.block<3, 3>(O_R, O_BG).cast<T>();
        Eigen::Matrix<T,3,3> dv_dba = pre_integration->jacobian.block<3, 3>(O_V, O_BA).cast<T>();
        Eigen::Matrix<T,3,3> dv_dbg = pre_integration->jacobian.block<3, 3>(O_V, O_BG).cast<T>();//scale convet


      //  Eigen::Matrix<T,3,3> dp_dbg1 = dp_dba.cast<T>();

        // // Eigen::Matrix<T,3,3> dp_dbg = pre_integration->jacobian.block<3, 3>(O_P, O_BG);
        // // Eigen::Matrix<T,3,3> dq_dbg = pre_integration->jacobian.block<3, 3>(O_R, O_BG);
        // // Eigen::Matrix<T,3,3> dv_dba = pre_integration->jacobian.block<3, 3>(O_V, O_BA);
        // // Eigen::Matrix<T,3,3> dv_dbg = pre_integration->jacobian.block<3, 3>(O_V, O_BG);
        // // 
        // // 
       // Eigen::Vector3d dba (static_cast<double>(Bai[0])-pre_integration->linearized_ba[0],static_cast<double>(Bai[1])-pre_integration->linearized_ba[1],static_const<double>(Bai[2])-pre_integration->linearized_ba[2]);
        //Eigen::Vector3d dbg = double(Bgi) - pre_integration->linearized_bg;

        // Eigen::Matrix<T,3,1> dba(Bai[0]-T(pre_integration->linearized_ba[0]),Bai[1]-T(pre_integration->linearized_ba[1]),Bai[2]-T(pre_integration->linearized_ba[2]));
        // Eigen::Matrix<T,3,1> dbg(Bai[0]-T(pre_integration->linearized_bg[0]),Bai[1]-T(pre_integration->linearized_bg[1]),Bai[2]-T(pre_integration->linearized_bg[2]));


        Eigen::Matrix<T,3,1> dba = Bai - pre_integration->linearized_ba.cast<T>();
        Eigen::Matrix<T,3,1> dbg = Bgi - pre_integration->linearized_bg.cast<T>();

        
        Eigen::Matrix<T,3,1> theta  = dq_dbg * dbg;
        //Eigen::Vector3d theta  = dq_dbg * dbg;
        T delta_theta[4] = {T(1) , T(theta[0]/T(2)),T(theta[1]/T(2)),T(theta[2]/T(2))};
        T delta_q1[4] = {T(pre_integration->delta_q.w()),T(pre_integration->delta_q.x()),T(pre_integration->delta_q.y()),T(pre_integration->delta_q.z())};
        T corrected_delta_q_temp[4] ;
        ceres::QuaternionProduct( delta_q1, delta_theta, corrected_delta_q_temp);
       // Eigen::Quaternion<T>  corrected_delta_q(corrected_delta_q_temp[3], corrected_delta_q_temp[0], corrected_delta_q_temp[1], corrected_delta_q_temp[2]);
        Eigen::Quaternion<T>  corrected_delta_q(corrected_delta_q_temp[0], corrected_delta_q_temp[1], corrected_delta_q_temp[2],corrected_delta_q_temp[3]);

        // // //Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * dbg);
        // Eigen::Vector3d corrected_delta_v = pre_integration->delta_v + dv_dba * dba + dv_dbg * dbg;
        // Eigen::Vector3d corrected_delta_p = pre_integration->delta_p + dp_dba * dba + dp_dbg * dbg;

        Eigen::Matrix<T,3,1> corrected_delta_v = pre_integration->delta_v.cast<T>() + dv_dba * dba + dv_dbg * dbg;
        Eigen::Matrix<T,3,1> corrected_delta_p = pre_integration->delta_p.cast<T>() + dp_dba * dba + dp_dbg * dbg;

        // cout <<"corrected_delta_v" <<corrected_delta_v[0] <<corrected_delta_v[1]<<corrected_delta_v[2]<< endl;
        // cout <<"corrected_delta_q" <<corrected_delta_q[0] <<corrected_delta_q[1]<<corrected_delta_q[2]<< endl;
        T sumt = T(pre_integration->sum_dt);

        //Eigen::Vector3d temp_P =0.5 * G * pre_integration->sum_dt ;

        Eigen::Matrix<T,3,1> temp_P = G.cast<T>() * sumt * sumt  / T(2)+ Pj - Pi - Vi * sumt ;

        T p1[3] = {temp_P[0],temp_P[1],temp_P[2]};

        T qi_inverse[4] = {Qi_inverse.w() , Qi_inverse.x() ,Qi_inverse.y() ,Qi_inverse.z()};

        T corrected_delta_p_temp[3] = {corrected_delta_p[0], corrected_delta_p[1], corrected_delta_p[2]};

        T r1[3];

        ceres::QuaternionRotatePoint(qi_inverse ,p1 , r1 );



        residual[0] = r1[0] - corrected_delta_p[0];
        residual[1] = r1[1]- corrected_delta_p[1];
        residual[2] = r1[2]- corrected_delta_p[2];




        Eigen::Quaternion<T>  corrected_delta_q_inverse = corrected_delta_q.inverse();

        T corrected_delta_q_inverse_temp[4] = {T(corrected_delta_q_inverse.w()), T(corrected_delta_q_inverse.x()), T(corrected_delta_q_inverse.y()), T(corrected_delta_q_inverse.z())};
        T qj[4] = {T(Qj.w()),T(Qj.x()),T(Qj.y()),T(Qj.z())};
        T temp_q1[4];
        ceres::QuaternionProduct(qi_inverse, qj, temp_q1 );
        T temp_q2[4];
        ceres::QuaternionProduct(corrected_delta_q_inverse_temp ,  temp_q1 , temp_q2 );
        

        residual[3] = T(2)*temp_q2[1];
        residual[4] = T(2)*temp_q2[2];
        residual[5] = T(2)*temp_q2[3];

        // T r2[3];
        
        // ceres::QuaternionToAngleAxis(temp_q2, r2);

        // residual[3] = T(2)*r2[0];
        // residual[4] = T(2)*r2[1];
        // residual[5] = T(2)*r2[2];

        // cout<<"temp_q2"<<temp_q2[0]<<temp_q2[1]<<temp_q2[2]<<endl;


        // cout<<"3 4 5"<<residual[3]<<residual[4]<<residual[5]<<endl;




        Eigen::Matrix<T,3,1> temp_V = G.cast<T>() * sumt + Vj - Vi;
        T v1[3] = {temp_V[0], temp_V[1], temp_V[2]};
        T temp_v1[3] ;
        ceres::QuaternionRotatePoint(qi_inverse ,v1 , temp_v1 );
        T r3[3] = {temp_v1[0] - corrected_delta_v[0], temp_v1[1] - T(corrected_delta_v[1]), temp_v1[2] - T(corrected_delta_v[2])};
        residual[6] = r3[0];
        residual[7] = r3[1];
        residual[8] = r3[2];



        T r4[3] = {Baj[0] - Bai[0], Baj[1] - Bai[1], Baj[2] - Bai[2]};
        residual[9] = r4[0];
        residual[10] = r4[1];
        residual[11] = r4[2];


        T r5[3] = {Bgj[0] - Bgi[0], Bgj[1] - Bgi[1], Bgj[2] - Bgi[2]};
        residual[12] = r5[0];
        residual[13] = r5[1];
        residual[14] = r5[2];

        


        Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration->covariance.inverse()).matrixL().transpose();

        Eigen::Matrix<T, 15, 15> sqrt_info_T = sqrt_info.cast<T>();


        residual = sqrt_info_T * residual;

       // cout<<"sqrt_info"<<sqrt_info<<endl;
        //
        // for(int i=0; i<15; i++)
        // {

        // cout<<"residual"<<i<<": "<<residual[i]<<endl;

        // }
        


        //residual.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * G * T(pre_integration->sum_dt) * T(pre_integration->sum_dt) + Pj - Pi - Vi * T(pre_integration->sum_dt)) - corrected_delta_p;
         //residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
        // residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_delta_v;
        // residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
        // residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;





        // Eigen::Vector3d dba = Bai - linearized_ba;
        // Eigen::Vector3d dbg = Bgi - linearized_bg;

    
        // ceres::QuaternionRotatePoint( q, point, p);
    

        return true;
    }
};


/*
struct PointToPointError_SophusSE3{
    const Eigen::Vector3d& p_dst;
    const Eigen::Vector3d& p_src;

    PointToPointError_SophusSE3(const Eigen::Vector3d &dst, const Eigen::Vector3d &src) :
        p_dst(dst), p_src(src)
    {
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d &observed, const Eigen::Vector3d &worldPoint) {
        return (new ceres::AutoDiffCostFunction<PointToPointError_SophusSE3, 3, 7>(new PointToPointError_SophusSE3(observed, worldPoint)));
    }

    template <typename T>
    bool operator()(const T* const cam1, T* residuals) const {

        //ceres::AngleAxisRotatePoint

        // Make sure the Eigen::Vector world point is using the ceres::Jet type as it's Scalar type
        Eigen::Matrix<T,3,1> p; p << T(p_src[0]), T(p_src[1]), T(p_src[2]);

        // Map the T* array to an Sophus SE3 object (with appropriate Scalar type)
        Sophus::SE3Group<T> q = Eigen::Map< const Sophus::SE3Group<T> >(cam1);

        // Rotate the point using Eigen rotations
        p = q.unit_quaternion() * p + q.translation();

        // The error is the difference between the predicted and observed position.
        residuals[0] = p[0] - T(p_dst[0]);
        residuals[1] = p[1] - T(p_dst[1]);
        residuals[2] = p[2] - T(p_dst[2]);

        return true;
    }
};
*/
struct PointToPlaneError_CeresAngleAxis{
    const Eigen::Vector3d& p_dst;
    const Eigen::Vector3d& p_src;
    const Eigen::Vector3d& p_nor;


    PointToPlaneError_CeresAngleAxis(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const Eigen::Vector3d& nor) :
    p_dst(dst), p_src(src), p_nor(nor)
    {
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d& observed, const Eigen::Vector3d& worldPoint, const Eigen::Vector3d& normal) {
        return (new ceres::AutoDiffCostFunction<PointToPlaneError_CeresAngleAxis, 1, 6>(new PointToPlaneError_CeresAngleAxis(observed, worldPoint,normal)));
    }

    template <typename T>
    bool operator()(const T* const camera, T* residuals) const {

        T p[3] = {T(p_src[0]), T(p_src[1]), T(p_src[2])};
        ceres::AngleAxisRotatePoint(camera,p,p);

        // camera[3,4,5] are the translation.
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        // The error is the difference between the predicted and observed position.
        residuals[0] = (p[0] - T(p_dst[0])) * T(p_nor[0]) + \
                       (p[1] - T(p_dst[1])) * T(p_nor[1]) + \
                       (p[2] - T(p_dst[2])) * T(p_nor[2]);

        return true;
    }
};

struct PointToPlaneError_EigenQuaternion{
    const Eigen::Vector3d& p_dst;
    const Eigen::Vector3d& p_src;
    const Eigen::Vector3d& p_nor;


    PointToPlaneError_EigenQuaternion(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const Eigen::Vector3d& nor) :
    p_dst(dst), p_src(src), p_nor(nor)
    {
    }


    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d& observed, const Eigen::Vector3d& worldPoint, const Eigen::Vector3d& normal) {
        return (new ceres::AutoDiffCostFunction<PointToPlaneError_EigenQuaternion, 1, 4, 3>(new PointToPlaneError_EigenQuaternion(observed, worldPoint,normal)));
    }

    template <typename T>
    bool operator()(const T* const camera_rotation, const T* const camera_translation, T* residuals) const {

        // Make sure the Eigen::Vector world point is using the ceres::Jet type as it's Scalar type
        Eigen::Matrix<T,3,1> point; point << T((double) p_src[0]), T((double) p_src[1]), T((double) p_src[2]);

        // Map the T* array to an Eigen Quaternion object (with appropriate Scalar type)
        Eigen::Quaternion<T> q = Eigen::Map<const Eigen::Quaternion<T> >(camera_rotation);

        // Rotate the point using Eigen rotations
        Eigen::Matrix<T,3,1> p = q * point;

        // Map T* to Eigen Vector3 with correct Scalar type
        Eigen::Matrix<T,3,1> t = Eigen::Map<const Eigen::Matrix<T,3,1> >(camera_translation);
        p += t;

        Eigen::Matrix<T,3,1> point2; point2 << T((double) p_dst[0]), T((double) p_dst[1]), T((double) p_dst[2]);
        Eigen::Matrix<T,3,1> normal; normal << T((double) p_nor[0]), T((double) p_nor[1]), T((double) p_nor[2]);

        // The error is the difference between the predicted and observed position projected onto normal
        residuals[0] = (p - point2).dot(normal);

        return true;
    }
};

/*struct PointToPlaneError_SophusSE3{
    const Eigen::Vector3d& p_dst;
    const Eigen::Vector3d& p_src;
    const Eigen::Vector3d& p_nor;


    PointToPlaneError_SophusSE3(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const Eigen::Vector3d& nor) :
    p_dst(dst), p_src(src), p_nor(nor)
    {
    }


    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d& observed, const Eigen::Vector3d& worldPoint, const Eigen::Vector3d& normal) {
        return (new ceres::AutoDiffCostFunction<PointToPlaneError_SophusSE3, 1, 7>(new PointToPlaneError_SophusSE3(observed, worldPoint,normal)));
    }

    template <typename T>
    bool operator()(const T* const cam1, T* residuals) const {

        // Make sure the Eigen::Vector world point is using the ceres::Jet type as it's Scalar type
        Eigen::Matrix<T,3,1> p; p << T(p_src[0]), T(p_src[1]), T(p_src[2]);
        Eigen::Matrix<T,3,1> point2; point2 << T(p_dst[0]), T(p_dst[1]), T(p_dst[2]);
        Eigen::Matrix<T,3,1> normal; normal << T(p_nor[0]), T(p_nor[1]), T(p_nor[2]);


        // Map the T* array to an Sophus SE3 object (with appropriate Scalar type)
        Sophus::SE3Group<T> q = Eigen::Map< const Sophus::SE3Group<T> >(cam1);

        // Rotate the point using Eigen rotations
        p = q.unit_quaternion() * p + q.translation();

        // The error is the difference between the predicted and observed position projected onto normal
        residuals[0] = (p - point2).dot(normal);

        return true;
    }
};*/

}
#endif // ICPCERES
