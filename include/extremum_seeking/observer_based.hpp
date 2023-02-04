#include <ros/ros.h>
#include <iostream>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <mavros_msgs/PositionTarget.h>

#include <random>
#include <Eigen/Eigen>

class ObserverBased{
    public:
    // Class Constructor
    ObserverBased(ros::NodeHandle& nh);

    // Class Destructor
    ~ObserverBased();

    private:
    ros::Subscriber gazebo_sub, odom_sub;
    ros::Publisher position_pub, pose_pub, ref_pub, targetPose_pub;

    // Variables
    double d = 0.0;
    bool takeoff = false;

    // Memory Variables
    Eigen::Matrix<double, 2, 1> hat_pts = Eigen::MatrixXd::Zero(2, 1);
    Eigen::Matrix<double, 3, 1> hat_z = Eigen::MatrixXd::Ones(3, 1)*0.1;
    Eigen::Matrix<double, 3, 3> P = Eigen::MatrixXd::Identity(3, 3);

    ros::Time previousTime = ros::Time::now();
    ros::Time initTime = ros::Time::now();

    // Constant
    double X_REF = -15; //34 - 49 // -15
    double Y_REF = -25; //24 - 49 // -25
    double Z_REF = 17;

    double THE_REF = -0.0926; //10*M_PI/180; //
    double PSI_REF = 3.9270; //155*M_PI/180; //
    double PHI_REF = 0; //0;

    double TAKEOFF_ALTITUDE = 6;

    double A = 2.5;
    double vMax = 3.0;
    double k = 0.1;
    double r = 1;
    double alpha = 0.1; //0.1

    Eigen::Matrix<double, 3, 3> R_IS;
    Eigen::Matrix<double, 2, 1> p_ref_s_xy_prev;

    void gazeboCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void updateObserver(Eigen::Matrix<double, 3, 1> p_di);
};