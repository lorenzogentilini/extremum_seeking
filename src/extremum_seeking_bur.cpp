#include <extremum_seeking/extremum_seeking_bur.hpp>

// Class Constructor
ExtremumSeekingBUR::ExtremumSeekingBUR(ros::NodeHandle& nh):
  gazebo_sub(nh.subscribe("/gazebo/model_states", 1, &ExtremumSeekingBUR::gazeboCallback, this)),
  odom_sub(nh.subscribe("/mavros/local_position/odom", 1, &ExtremumSeekingBUR::odometryCallback, this)),
  arva_sub(nh.subscribe("/arva", 1, &ExtremumSeekingBUR::arvaCallback, this)),
  position_pub(nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1)),
  pose_pub(nh.advertise<geometry_msgs::PointStamped>("/pose", 1)),
  signal_pub(nh.advertise<std_msgs::Float64>("/signal", 1)),
  arvaTimer(nh.createTimer(ros::Duration(0.05), &ExtremumSeekingBUR::updateArva, this)),
  executeTimer(nh.createTimer(ros::Duration(0.1), &ExtremumSeekingBUR::execute, this)){
    
  iRp(0,0) = 0.5770;
	iRp(0,1) = -0.7071;
	iRp(0,2) = -0.4087;

	iRp(1,0) = 0.5770;
	iRp(1,1) = 0.7071;
	iRp(1,2) = -0.4087;

	iRp(2,0) = 0.5780;
	iRp(2,1) = 0.0;
	iRp(2,2) = 0.8161;

	Op(0,0) = 0.0;
	Op(1,0) = 0.0;
	Op(2,0) = 6.1268;

  initClient(IPaddress, ServerPort);
}

// Class Destructor
ExtremumSeekingBUR::~ExtremumSeekingBUR(){
  ;
}

void ExtremumSeekingBUR::execute(const ros::TimerEvent& e){
  if(!arvaValid){
    return;
  }

  if(!takeoff){
    initTime = ros::Time::now();

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
    msg_pub.yaw = YAW_SETPOINT;
    position_pub.publish(msg_pub);

    return;
  }

  // Extremum Seeking BUR
  Eigen::Matrix<double, 3, 1> sp_p = update();
  //Eigen::Matrix<double, 3, 1> sp_i = Op + iRp*sp_p;

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

  // msg_pub.position.x = sp_p(0)*cos(yy) + sp_p(1)*sin(yy);
  // msg_pub.position.y = -sp_p(0)*sin(yy) + sp_p(1)*cos(yy);
  msg_pub.position.x = sp_p(0);
  msg_pub.position.y = sp_p(1);
  msg_pub.position.z = TAKEOFF_ALTITUDE;
  msg_pub.yaw = YAW_SETPOINT;
  position_pub.publish(msg_pub);
}

Eigen::Matrix<double, 3, 1> ExtremumSeekingBUR::update(){
  Eigen::Matrix<double, 3, 1> x;
	
	// fPlus = FA_BUR*f + FB_BUR*ALPHA_BUR;
  // double alpha = FC_BUR*f + FD_BUR*ALPHA_BUR;

	double time = (ros::Time::now() - initTime).toSec();
	double deltaTime = time - previousTime;

	// X - Direction
  x(0) = xOld + (sqrt(ALPHA_BUR*OMEGA_BUR)*cos(OMEGA_BUR*time + KAPPA_BUR*_arva_filtered))*deltaTime;

  // Y - Direction
  x(1) = yOld + (sqrt(ALPHA_BUR*OMEGA_BUR)*sin(OMEGA_BUR*time + KAPPA_BUR*_arva_filtered))*deltaTime;

	// Z - Direction
	x(2) = 0.0;

	// Update Memory Variables
  // f = fPlus;
	xOld = x(0);
	yOld = x(1);
	previousTime = time;

	return x;
}

void ExtremumSeekingBUR::updateArva(const ros::TimerEvent& e){
	double Range = 0;
	double RSSI = 0;
	int* ReceiveHashtag = NULL;
	long long int IDTarget;
	int result = 0;

  int TX_Hashcode = rand();
  sprintf(TX_Buffer, "{ %d %d %llX}", TX_Hashcode, CLIENT_GET_RANGE, UniqueID);
  sendData(TX_Buffer, strlen(TX_Buffer), &addrService);
  if(receiveData(0, Timeout*1000) < 0){
    ROS_WARN("Comunication Timeout!!");
  } else{
    RX = messagge();
    RemoveString(RX, PACKET_START_DELIMETER);
    RemoveString(RX, PACKET_END_DELIMETER);

    sscanf(RX, "%d %d %lf %llX %lf", &ReceiveHashtag, &result, &Range, &IDTarget, &RSSI);
    if(result == 255) {
      ROS_WARN("Ranging Error!!");
    } else{
      double sample = (double)Range;
      if(use_rssi){
        sample = (double)RSSI;
      }

      if(samples.size() < n) {
        samples.push_back(sample);
      } else{
        partial_sum -= samples[0];
        samples.erase(samples.begin());
        samples.push_back(sample);
      }
      
      partial_sum += sample;
      y = partial_sum/(double)samples.size();
    }
  }

  if(use_rssi){
    y = pow(10, y/20.0);
    y = 0.0001/y;
  }

  if(isFirstTime){
    ff = ((1-FD_ARVA)*y)/FC_ARVA;
    isFirstTime = false;
  }

  // Update ARVA
  ffPlus = FA_ARVA*ff + FB_ARVA*y;
  _arva_filtered = FC_ARVA*ff + FD_ARVA*y;
  ff = ffPlus;

  std_msgs::Float64 msg_;
  msg_.data = _arva_filtered;
  signal_pub.publish(msg_);

  ROS_INFO("Data From UWB %f", _arva_filtered);

  arvaValid = true;
}

void ExtremumSeekingBUR::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
  // Saving Current Position
  actualPosition << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;

  if(abs(msg->pose.pose.position.z - TAKEOFF_ALTITUDE) < 0.2){
    takeoff = true;
  }
}

void ExtremumSeekingBUR::arvaCallback(const std_msgs::Float64::ConstPtr& msg){
  double y = msg->data;
  
  if(isFirstTime){
    ff = ((1-FD_ARVA)*y)/FC_ARVA;
    isFirstTime = false;
  }

  // Update ARVA
  ffPlus = FA_ARVA*ff + FB_ARVA*y;
  _arva_filtered = FC_ARVA*ff + FD_ARVA*y;
  ff = ffPlus;

  std_msgs::Float64 msg_;
  msg_.data = _arva_filtered;
  signal_pub.publish(msg_);

  ROS_INFO("Data From ARVA Sensor %f", _arva_filtered);

  arvaValid = true;
}

// Substitute That With the Sensor Callback
void ExtremumSeekingBUR::gazeboCallback(const gazebo_msgs::ModelStates::ConstPtr& msg){
  double xActual, yActual, zActual;

  // Select the correct ID
  xActual = msg->pose[2].position.x;
  yActual = msg->pose[2].position.y;
  zActual = msg->pose[2].position.z;

  geometry_msgs::PointStamped pose_msgs;
  pose_msgs.header.stamp = ros::Time::now();
  pose_msgs.point.x = xActual;
  pose_msgs.point.y = yActual;
  pose_msgs.point.z = zActual;
  pose_pub.publish(pose_msgs);

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

  // H += hNoise*nNoise*realDistribution(realGenerator);

  // Compute ARVA Function
  //y = cbrt(1/H.norm());
  //y = cbrt(pow(1/(4*M_PI*H.norm()), 2));
  //y = cbrt(1/(4*M_PI*H.norm()));

  y = p.norm();

  if(isFirstTime){
    ff = ((1-FD_ARVA)*y)/FC_ARVA;
    isFirstTime = false;
  }

  // Update ARVA
  ffPlus = FA_ARVA*ff + FB_ARVA*y;
  _arva_filtered = FC_ARVA*ff + FD_ARVA*y;
  ff = ffPlus;

  std_msgs::Float64 msg_;
  msg_.data = _arva_filtered;
  signal_pub.publish(msg_);

  ROS_INFO("Data From Gazebo %f", _arva_filtered);

  arvaValid = true;
}

// EXTERNAL FUNCTIONS - FOR REAL EXPERIMENTS 
int ExtremumSeekingBUR::RemoveString(char *s, char *string_to_remove){
  int found = 0;
  char *p;

  p = strstr(s, string_to_remove);
  if(p != NULL){
    found = 1; strcpy(p, p + strlen(string_to_remove));
  }

  return found;
}

void ExtremumSeekingBUR::closeSocket(){
  close(socketService);
}

int ExtremumSeekingBUR::initClient(char *ip_add, unsigned int port){
  //closeSocket();
  socketService = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (socketService == -1){
    return -1;
  }

  addrService.sin_family = AF_INET;
  addrService.sin_port = htons(port);

  if (inet_aton(ip_add, &addrService.sin_addr) == 0){
    return -2;
  }
  
  maxfd = socketService;
  FD_ZERO(&static_rdset);
  FD_SET(socketService, &static_rdset);
  return 0;
}

int ExtremumSeekingBUR::receiveData(int timeout_sec, int timeout_us){
  struct timeval timeout = { timeout_sec,timeout_us }; //set timeout
  // set receive UDP message timeout
  setsockopt(socketService, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(struct timeval));
  
  socklen_t size;
  size = sizeof(addrService);
  bytesRecv = recvfrom(socketService, recvbuf, UDP_BUFFER_LENGTH - 1, 0, (struct sockaddr *) &addrService, &size);
  if(bytesRecv <= 0){
    return -1;
  }

  recvbuf[bytesRecv] = 0;
  return 0;
}

int ExtremumSeekingBUR::sendData(char* buffer, int length, sockaddr_in *destaddr){
  int size = sizeof(struct sockaddr);
  if(destaddr == NULL)
    destaddr = &addrService;

  bytesSent = sendto(socketService, buffer, length, 0, (struct sockaddr *) destaddr, size);
  if(bytesSent <= 0){
    return -1;
  }
  
  return 0;
}

char* ExtremumSeekingBUR::messagge(){
  return recvbuf;
}