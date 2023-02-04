#include <extremum_seeking/observer_based.hpp>

// Class Constructor
ObserverBased::ObserverBased(ros::NodeHandle& nh):
  gazebo_sub(nh.subscribe("/gazebo/model_states", 1, &ObserverBased::gazeboCallback, this)),
  odom_sub(nh.subscribe("/mavros/local_position/odom", 1, &ObserverBased::odometryCallback, this)),
  position_pub(nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1)),
  pose_pub(nh.advertise<geometry_msgs::PointStamped>("/pose", 1)),
  ref_pub(nh.advertise<geometry_msgs::PointStamped>("/ref", 1)),
  targetPose_pub(nh.advertise<geometry_msgs::PointStamped>("/target", 1)){
  
  hat_z << 1, 1, 1, 1, 1;

  R_IS(0,0) = 0.5770;
  R_IS(0,1) = -0.7071;
  R_IS(0,2) = -0.4087;

  R_IS(1,0) = 0.5770;
  R_IS(1,1) = 0.7071;
  R_IS(1,2) = -0.4087;

  R_IS(2,0) = 0.5780;
  R_IS(2,1) = 0.0;
  R_IS(2,2) = 0.8161;
}

// Class Destructor
ObserverBased::~ObserverBased(){
  ;
}

void ObserverBased::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
  Eigen::Matrix<double, 3, 1> p_di;
  p_di << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  
  Eigen::Matrix<double, 3, 1> p_ds = R_IS.transpose()*p_di;
  Eigen::Matrix<double, 2, 1> p_ds_xy;
  p_ds_xy << p_ds(0), p_ds(1);

  if(!takeoff){
    initTime = ros::Time::now();
    p_ref_s_xy_prev = p_ds_xy;

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

  // Update Observer
  updateObserver(p_di);

  Eigen::Matrix<double, 2, 1> VSxy;
  VSxy = alpha*((vMax*k*d)/sqrt(1+pow(k*d,2)))*(hat_pts - p_ds_xy).normalized();

  ros::Time actualTime = ros::Time::now();
  double delta_time = (actualTime - previousTime).toSec();
  previousTime = actualTime;

  Eigen::Matrix<double, 2, 1> p_ref_s_xy = p_ref_s_xy_prev + VSxy*delta_time;
  p_ref_s_xy_prev = p_ref_s_xy;

  Eigen::Matrix<double, 2, 1> p_pert;

  double w = vMax*(1-alpha)/(A*sqrt(5));
  p_pert << A*sin(w*(actualTime - initTime).toSec()), A*sin(2*w*(actualTime - initTime).toSec());

  p_ref_s_xy = p_ref_s_xy + p_pert;
  Eigen::Matrix<double, 3, 1> p_ref_s, p_ref_i;
  p_ref_s << p_ref_s_xy(0), p_ref_s_xy(1), 0;
  p_ref_i = R_IS*p_ref_s;

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
  msg_pub.position.x = p_ref_i(0);
  msg_pub.position.y = p_ref_i(1);
  msg_pub.position.z = p_ref_i(2) + 6;
  msg_pub.yaw = 0.0;
  position_pub.publish(msg_pub);

  geometry_msgs::PointStamped ref_msgs;
  ref_msgs.header.stamp = ros::Time::now();
  ref_msgs.point.x = p_ref_i(0);
  ref_msgs.point.y = p_ref_i(1);
  ref_msgs.point.z = p_ref_i(2) + 6;
  ref_pub.publish(ref_msgs);
}

void ObserverBased::gazeboCallback(const gazebo_msgs::ModelStates::ConstPtr& msg){
  double xActual, yActual, zActual;

  // Select the correct ID
  xActual = msg->pose[1].position.x;
  yActual = msg->pose[1].position.y;
  zActual = msg->pose[1].position.z;

  geometry_msgs::PointStamped pose_msgs;
  pose_msgs.header.stamp = ros::Time::now();
  pose_msgs.point.x = xActual;
  pose_msgs.point.y = yActual;
  pose_msgs.point.z = zActual;
  pose_pub.publish(pose_msgs);

  if(abs(zActual - TAKEOFF_ALTITUDE) < 0.2){
    takeoff = true;
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

  d = cbrt(pow(1/(4*M_PI*H.norm()), 2));
}

void ObserverBased::updateObserver(Eigen::Matrix<double, 3, 1> p_di){
  Eigen::Matrix<double, 3, 1> p_ds = R_IS.transpose()*(p_di - Eigen::Matrix<double, 3, 1>(0,0,6));
  Eigen::Matrix<double, 2, 1> p_ds_xy;
  p_ds_xy << p_ds(0), p_ds(1);

  Eigen::Matrix<double, 1, 3> hat_c;
  hat_c << -2*p_ds(0), -2*p_ds(1), 1;

  double hat_y = (hat_c*hat_z)(0);

  P = 0.5*(P + P.transpose());
  double norm = r + (hat_c*P*hat_c.transpose());
  Eigen::Matrix<double, 3, 1> K = P*hat_c.transpose()/norm;

  double y = d - p_ds_xy.squaredNorm();

  hat_z = hat_z + (K*(y - hat_y));
  P = ((Eigen::MatrixXd::Identity(3, 3) - K*hat_c)*P*((Eigen::MatrixXd::Identity(3, 3) - K*hat_c).transpose())) + Eigen::MatrixXd::Identity(3, 3) + K*r*K.transpose();
  P = 0.5*(P + P.transpose());

  hat_pts(0) = hat_z(0);
  hat_pts(1) = hat_z(1);

  Eigen::Matrix<double, 3, 1> targetPose_est;
  targetPose_est << hat_pts(0), hat_pts(1), 0;
  targetPose_est = R_IS*targetPose_est;

  geometry_msgs::PointStamped tearget_msgs;
  tearget_msgs.header.stamp = ros::Time::now();
  tearget_msgs.point.x = targetPose_est(0);
  tearget_msgs.point.y = targetPose_est(1);
  tearget_msgs.point.z = targetPose_est(2) + 6;
  targetPose_pub.publish(tearget_msgs);
}
