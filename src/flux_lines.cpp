#include <extremum_seeking/flux_lines.hpp>

// Class Constructor
FluxLines::FluxLines(ros::NodeHandle& nh):
  gazebo_sub(nh.subscribe("/gazebo/model_states", 1, &FluxLines::gazeboCallback, this)),
  setpoint_pub(nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1)),
  position_pub(nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1)),
  pose_pub(nh.advertise<geometry_msgs::PointStamped>("/pose", 1)),
  field_pub(nh.advertise<geometry_msgs::PointStamped>("/field", 1)){
  ;
}

// Class Destructor
FluxLines::~FluxLines(){
  ;
}

void FluxLines::gazeboCallback(const gazebo_msgs::ModelStates::ConstPtr& msg){
  double xActual, yActual, zActual;

  // Select the correct ID
  xActual = msg->pose[1].position.x;
  yActual = msg->pose[1].position.y;
  zActual = msg->pose[1].position.z;

  geometry_msgs::PointStamped pose_msgs;
  pose_msgs.point.x = xActual;
  pose_msgs.point.y = yActual;
  pose_msgs.point.z = zActual;
  pose_msgs.header.stamp = ros::Time::now();
  pose_pub.publish(pose_msgs);

  if(abs(zActual - TAKEOFF_ALTITUDE) < 0.2){
    takeoff = true;
  }

  if(!takeoff){
    mavros_msgs::PositionTarget msg_pub;
    msg_pub.header.stamp = ros::Time::now();
    msg_pub.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    msg_pub.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                        mavros_msgs::PositionTarget::IGNORE_VY |
                        mavros_msgs::PositionTarget::IGNORE_VZ |
                        mavros_msgs::PositionTarget::IGNORE_AFX |
                        mavros_msgs::PositionTarget::IGNORE_AFY |
                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    msg_pub.position.x = 0.0;
    msg_pub.position.y = 0.0;
    msg_pub.position.z = TAKEOFF_ALTITUDE;
    msg_pub.yaw = 0.0;
    position_pub.publish(msg_pub);

    return;
  }

  Eigen::Matrix<double, 3, 1> p(xActual - X_REF, yActual - Y_REF, zActual - Z_REF);

  // Build Matrices Rt, Rr, A and e1
  Eigen::Matrix<double, 3, 3> Rt, Rx, Ry, Rz, A;
  Eigen::Matrix<double, 3, 1> e1 (1, 0, 0);

  Rx << 1, 0, 0,
        0, cos(PHI_REF), sin(PHI_REF),
        0, -sin(PHI_REF), cos(PHI_REF);

  Ry << cos(THE_REF), 0, -sin(THE_REF),
        0, 1, 0,
        sin(THE_REF), 0, cos(THE_REF);

  Rz << cos(PSI_REF), sin(PSI_REF), 0,
        -sin(PSI_REF), cos(PSI_REF), 0,
        0, 0, 1;

  // From Transmitter to Inertial
  // FIRST Rotation PHI - SECOND THETA - THIRD PSI
  Rt = (Rx*Ry*Rz).transpose();

  A << 2*pow(p(0), 2)-pow(p(1), 2)-pow(p(2), 2), 3*p(0)*p(1), 3*p(0)*p(2),
       3*p(0)*p(1), 2*pow(p(1), 2)-pow(p(0), 2)-pow(p(2), 2), 3*p(1)*p(2),
       3*p(0)*p(2), 3*p(1)*p(2), 2*pow(p(2), 2)-pow(p(0), 2)-pow(p(1), 2);

  // Compute Magnetic Dipole
  Eigen::Matrix<double, 3, 1> H;
  H = (A*Rt*e1)/(4*M_PI*pow(p.norm(), 5));

  // Add Floor Noise
  static std::default_random_engine realGenerator;
  static std::default_random_engine intGenerator;
  std::uniform_real_distribution<double>  realDistribution(-1.0,1.0);
  std::uniform_int_distribution<int>      intDistribution(0,1);

  Eigen::Matrix<double, 3, 1> hNoise;
  double kNoise, rNoise, nNoise, xNoise, yNoise, zNoise;

  kNoise = 1.5420;
  rNoise = 80;
  nNoise = kNoise/(4*M_PI*pow(rNoise, 3));

  xNoise = realDistribution(realGenerator);
  yNoise = realDistribution(realGenerator)*sqrt(1 - pow(xNoise, 2));
  zNoise = pow(-1, intDistribution(intGenerator))*sqrt(1- pow(xNoise, 2) - pow(yNoise, 2));

  hNoise << xNoise, yNoise, zNoise;

  H += hNoise*nNoise*realDistribution(realGenerator);

  // Compute Line Flux SetPoint
  Eigen::Matrix<double, 3, 3> R_IS, R_SI;
  R_IS(0,0) = 0.5770;
	R_IS(0,1) = -0.7071;
	R_IS(0,2) = -0.4087;

	R_IS(1,0) = 0.5770;
	R_IS(1,1) = 0.7071;
	R_IS(1,2) = -0.4087;

	R_IS(2,0) = 0.5780;
	R_IS(2,1) = 0.0;
	R_IS(2,2) = 0.8161;

  R_SI = R_IS.transpose();
  H = R_SI*H;

  Eigen::Matrix<double, 2, 1> HSxy;
  HSxy = H.block(0,0,2,1);
  double d = 1/HSxy.norm();

  Eigen::Matrix<double, 2, 1> VSxy;
  double vMax = 3.0;
  double k = 0.1;
  VSxy = ((vMax*k*d)/sqrt(1+pow(k*d,2)))*HSxy.normalized();

  Eigen::Matrix<double, 3, 1> VI, VSxyz;
  VSxyz << VSxy(0), VSxy(1), 0;
  VI = R_IS*VSxyz;

  geometry_msgs::TwistStamped msg_vel;
  msg_vel.twist.linear.x = VI(0);
  msg_vel.twist.linear.y = VI(1);
  msg_vel.twist.linear.z = VI(2);
  msg_vel.header.stamp = ros::Time::now();
  msg_vel.header.frame_id = "map";
  setpoint_pub.publish(msg_vel);

  geometry_msgs::PointStamped field_msgs;
  field_msgs.point.x = H(0);
  field_msgs.point.y = H(1);
  field_msgs.point.z = H(2);
  field_msgs.header.stamp = ros::Time::now();
  field_pub.publish(field_msgs);
}
