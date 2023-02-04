#include <ros/ros.h>

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <random>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Float64.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <linux/if_packet.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <netinet/ether.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <signal.h>
#include <time.h>
#include <ifaddrs.h>

#include <Eigen/Eigen>

#define CLIENT_PING  0
#define CLIENT_SCAN  56
#define CLIENT_GET_RANGE  50
#define PACKET_START_DELIMETER  "{"
#define PACKET_END_DELIMETER    "}"
#define UDP_BUFFER_LENGTH	2048

typedef struct {
	long long int ID;
	int RSSI;
} NeighboursList_t;

class ExtremumSeekingBUR{
    public:
    // Class Constructor
    ExtremumSeekingBUR(ros::NodeHandle& nh);

    // Class Destructor
    ~ExtremumSeekingBUR();

    private:
    ros::Subscriber gazebo_sub, odom_sub, arva_sub;
    ros::Publisher position_pub, pose_pub, signal_pub;
    ros::Timer executeTimer, arvaTimer;

    // UDP Variables
    int		maxfd;
    fd_set	static_rdset, rdset;
    int		bytesRecv;
    int		bytesSent;
    char	recvbuf[UDP_BUFFER_LENGTH];

    NeighboursList_t NodeList[56];
	char TX_Buffer[1024];
	int	Timeout = 2000;
	char * RX;
	char IPaddress[13] = "169.254.3.96";
	int ServerPort = 5678;
    long long int UniqueID = 0x10205EA710000D86; 
    int socketService;
    sockaddr_in addrService;
    double partial_sum = 0.0;

    uint n = 15;
	std::vector<double> samples;

    // Variables
    bool takeoff = false;
    bool arvaValid = false;
    bool use_rssi = false;

    // Constant
    double X_REF = 5; // -15
    double Y_REF = 2; // -25
    double Z_REF = 0; // 17

    double THE_REF = 10*M_PI/180;
    double PSI_REF = 155*M_PI/180;
    double PHI_REF = 0;

    double FA_ARVA = -0.2381;
    double FB_ARVA = 0.5;
    double FC_ARVA = 0.9433;
    double FD_ARVA = 0.6190;

    double FA_BUR = 0.9950;
    double FB_BUR = 0.0625;
    double FC_BUR = 0.0796;
    double FD_BUR = 0.0025;

    double vv = 0.4;
    double rr = 0.5;

    double OMEGA_BUR = vv/rr; //2.0; // v/r
    double ALPHA_BUR = vv*rr; //0.5; // v*r
    double KAPPA_BUR = 1.0; //1.0;

    double TAKEOFF_ALTITUDE = 3;
    double YAW_SETPOINT = 0.0;

    double y = 0, _arva_filtered = 0;
    double ffPlus = 0.0, ff = 0.0;
    double fPlus = 0.0, f = 0.0;
    double xOld = 0, yOld = 0;
    double previousTime = 0;
    
    Eigen::Matrix<double, 3, 1> actualPosition;
    Eigen::Matrix<double, 3, 3> iRp;
    Eigen::Matrix<double, 3, 1> Op;

    bool isFirstTime = true;

    ros::Time initTime;

    void gazeboCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void arvaCallback(const std_msgs::Float64::ConstPtr& msg);
    void execute(const ros::TimerEvent& e);
    void updateArva(const ros::TimerEvent& e);
    Eigen::Matrix<double, 3, 1> update();

    int RemoveString(char *s, char *string_to_remove);
    char* messagge();
    int sendData(char* buffer, int length, sockaddr_in *destaddr);
    int receiveData(int timeout_sec, int timeout_us);
    int initClient(char *ip_add, unsigned int port);
    void closeSocket();
};