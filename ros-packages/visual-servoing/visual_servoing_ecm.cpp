#include <algorithm>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

geometry_msgs::PoseStamped pose;
bool msg_received = false;
Eigen::Matrix<double,6,1> twist_des;
std::vector<double> joint_state_current;
Eigen::Matrix<double,3,3> R;
Eigen::Matrix<double,3,1> p;
Eigen::Matrix<double,3,1> rce;

void cartesianVelocityDesiredCallback(const geometry_msgs::Twist& msg)
{
  twist_des << msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z;
}

void objectPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  pose = msg;
}

void jointStateCallback(const sensor_msgs::JointState& msg)
{
    msg_received = true;
    joint_state_current.clear();
    for (unsigned i=0; i<msg.position.size(); i++)\
    {
        joint_state_current.push_back(msg.position[i]);
        std::cout << joint_state_current[i] <<std::endl;
    }
}

void forwardKinematicsECM(std::vector<double>& q)
{
  double q1 = q[0];
  double q2 = q[1];
  double q3 = q[2];
  double q4 = q[3];
  double t2 = cos(q1);
  double t3 = sin(q4);
  double t4 = cos(q4);
  double t5 = sin(q1);
  double t6 = sin(q2);
  double t7 = cos(q2);
  R(0,0) = t2*t4-t3*t5*t6;
  R(0,1) = -t2*t3-t4*t5*t6;
  R(0,2) = -t5*t7;
  p(0)   = -q3*t5*t7;
  R(1,0) = t4*t5+t2*t3*t6;
  R(1,1) = -t3*t5+t2*t4*t6;
  R(1,2) = t2*t7;
  p(1)   = q3*t2*t7;
  R(2,0) = -t3*t7;
  R(2,1) = -t4*t7;
  R(2,2) = t6;
  p(2)   = q3*t6;
}


Eigen::Matrix<double,6,4> ecm_jacobian(std::vector<double>& q)
{
  Eigen::Matrix<double,6,4> jacobian;
  jacobian.setZero();
  double q1 = q[0];
  double q2 = q[1];
  double q3 = q[2];
  double q4 = q[3];
  double t2 = cos(q2);
  double t3 = sin(q1);
  double t4 = cos(q1);
  double t5 = sin(q2);
  double t6 = t2*t4;
  jacobian(0,0) = -q3*t2*t4;
  jacobian(0,1) = q3*t3*t5;
  jacobian(0,2) = -t2*t3;
  jacobian(1,0) = -q3*t2*t3;
  jacobian(1,1) = -q3*t4*t5;
  jacobian(1,2) = t6;
  jacobian(2,1) = q3*t2*(t3*t3)+q3*t2*(t4*t4);
  jacobian(2,2) = t5;
  jacobian(3,1) = t4;
  jacobian(3,3) = -t2*t3;
  jacobian(4,1) = t3;
  jacobian(4,3) = t6;
  jacobian(5,0) = 1.0;
  jacobian(5,3) = t5;
  return jacobian;
}

template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
        {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
        else
        {
            singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}

Eigen::Matrix<double,6,1> invertTwist(Eigen::Matrix<double,6,1> twist)
{
  //twist should be given in [v,w] form
  Eigen::Matrix<double,6,1> invertedTwist;
  invertedTwist << twist[3], twist[4], twist[5], twist[0], twist[1], twist[2]; 
  return invertedTwist;   
}

Eigen::Matrix<double,6,1> rotateTwist(Eigen::Matrix<double,6,1> twist, Eigen::Matrix<double,3,3> R, Eigen::Matrix<double,3,1> p)
{
  //twist is given in [v,w] form and returned in the same form
  Eigen::Matrix<double,6,1> rotatedTwist;
  Eigen::Matrix<double,6,6> X;
  X.setZero();
  X.block<3,3>(0,0) = R;
  X.block<3,3>(3,3) = R;
  rotatedTwist = X*twist;

  Eigen::Matrix3d S;
  S << 0, -p[2], p[1], p[2], 0, -p[0], -p[1], p[0], 0;
  
  X.setIdentity();
  X.block<3,3>(0,3) = -S;
  rotatedTwist = (rotatedTwist.transpose()*X).transpose();
  return rotatedTwist;   
}

int main(int argc, char **argv)
{
    //Initiate the Ros Publishers
    
    ros::init(argc, argv, "ros_node_visual_servoing");
    ros::NodeHandle n_1;
    ros::Subscriber object_pose_sub = n_1.subscribe("object_1/pose", 1, objectPoseCallback);
    ros::Subscriber des_velocity_sub = n_1.subscribe("/dvrk/ECM/cartesian_velocity_desired", 1, cartesianVelocityDesiredCallback);
    ros::Subscriber joint_state_sub = n_1.subscribe("/dvrk/ECM/state_joint_current", 1, jointStateCallback);
    ros::Publisher joint_state_pub = n_1.advertise<sensor_msgs::JointState>("/dvrk/ECM/state_joint_desired", 1);
    
    ros::Rate loop_rate(100);
    
    sensor_msgs::JointState ecm_des_jnts;

    int count = 0;
    Eigen::Matrix<double,4,6> jacp;
    Eigen::Matrix3d K;
    K.setIdentity();
    K = 0.5*K; 
    std::vector<double> q_std;
    ros::Duration(1).sleep();
    rce << -0.0025, 0, -0.0035;
    //rce << 0, 0, 0;
    int iii=1;
    Eigen::Matrix<double,6,4> jacobian = Eigen::Matrix<double,6,4>::Zero();
    Eigen::Vector4d q_eigen = Eigen::Vector4d::Zero();
    Eigen::Matrix3d Rb;
    while(ros::ok())    
    {
        if (msg_received)
        {   
            //update robot generalized coordinates
            q_std = joint_state_current;

            Eigen::Vector4d q_init(q_std.data());
            forwardKinematicsECM(q_std);
            jacobian = ecm_jacobian(q_std);
            Eigen::Matrix<double,6,1> twist_base = rotateTwist(twist_des, R, rce);

            jacp = pseudoinverse(jacobian);

            Eigen::Vector4d dq = jacp*twist_base;
            q_eigen = q_init+dq*0.01;
            q_std.resize(q_eigen.size());

            Eigen::VectorXd::Map(&q_std[0],q_eigen.size()) = q_eigen;
            sensor_msgs::JointState joint_state_desired;
            joint_state_desired.position = q_std;
            joint_state_pub.publish(joint_state_desired);

        }
        ros::spinOnce();
        loop_rate.sleep();
    }
return 0;
}