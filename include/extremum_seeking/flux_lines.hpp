#include <ros/ros.h>
#include <iostream>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <mavros_msgs/PositionTarget.h>

#include <random>
#include <Eigen/Eigen>

class FluxLines{
    public:
    // Class Constructor
    FluxLines(ros::NodeHandle& nh);

    // Class Destructor
    ~FluxLines();

    private:
    ros::Subscriber gazebo_sub;
    ros::Publisher setpoint_pub, position_pub, pose_pub, field_pub;

    bool takeoff = false;

    double X_REF = -15; //34 - 49 // -15
    double Y_REF = -25; //24 - 49 // -25
    double Z_REF = 17;

    double THE_REF = -0.0926; // 10*M_PI/180;
    double PSI_REF = 3.9270; // 155*M_PI/180;
    double PHI_REF = 0;

    double TAKEOFF_ALTITUDE = 6;

    void gazeboCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
};