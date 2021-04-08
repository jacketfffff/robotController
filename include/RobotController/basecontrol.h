#ifndef WHEEL_FOOT_ROBOT_BASECONTROL_H
#define WHEEL_FOOT_ROBOT_BASECONTROL_H

#include "RobotController/NaviSerialManager.h"
#include "RobotController/WheelStatus.h"
#include "RobotController/JoyTeleop.h"
#include <unistd.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include "ros/ros.h"
#include "ros/timer.h"
#include "ros/wall_timer.h"

#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <climits>
#include <yaml-cpp/yaml.h>

#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
//#include <sys/time.h>



//#include <valarray>


//#include "sensor_msgs/Image.h"

//#include "opencv2/opencv.hpp"
//#include <cv_bridge/cv_bridge.h>
//#include "std_msgs/UInt32.h"
//#include "fstream"
//#include "chrono"
//#include <iostream>


class BaseController {
public:
    struct Encoder {
        int right_encoder;
        int left_encoder;
        double interval;
        bool encoderWrong;
    };
    struct Cmd_vel {
        char cmd_right_wheel;
        char cmd_left_wheel;
    }user_cmd_vel_{};
    struct Base_model {
        double Wheel_Diameter{};
        double Encoder_to_Distance{};
        double Wheel_Base{};
        double Wheel_Center_X_Offset{};
        double Wheel_Center_Y_Offset{};
    };
    enum Command{
        DEFAULT,
        RESET,
        INIT,
        CLEARRESET,
        SQUAT,
        STAND,
        KNEECUT,
        KNEELOCK,
        ANKLELOCK,
        ANKLECUT,
        HIPCUT,
        HIPLOCK,
        GET_POSE,
        GET_ENCODER,
        STOP,
		IMU
    }user_command_{};
private:
    //std::tr1::shared_ptr<boost::thread> thread_ptr_;
	//int tou,tou_1, zuo ,you;
    char wheel_vel[COMMAND_SIZE] = {0x53,0x13,0x10,0x03,0x00,0x00,0x00,0x00};
    unsigned char joint_init[COMMAND_SIZE] = {0x53,0x10,0x56,0x00,0x00,0x00,0x00,0x00};
    unsigned char joint_reset[COMMAND_SIZE] = {0x53,0x13,0x32,0xFF,0x00,0x00,0x00,0x00};

    unsigned char left_knee_cut[COMMAND_SIZE] = {0x53,0x10,0x10,0x04,0x00,0x00,0x00,0x00};
    unsigned char right_knee_cut[COMMAND_SIZE] = {0x53,0x10,0x10,0x14,0x00,0x00,0x00,0x00};
    unsigned char left_ankle_cut[COMMAND_SIZE] = {0x53,0x10,0x10,0x07,0x00,0x00,0x00,0x00};
    unsigned char right_ankle_cut[COMMAND_SIZE] = {0x53,0x10,0x10,0x17,0x00,0x00,0x00,0x00};
    unsigned char left_hip_cut[COMMAND_SIZE] = {0x53,0x10,0x10,0x02,0x00,0x00,0x00,0x00};
    unsigned char right_hip_cut[COMMAND_SIZE] = {0x53,0x10,0x10,0x12,0x00,0x00,0x00,0x00};

    unsigned char left_knee_lock[COMMAND_SIZE] = {0x53,0x10,0x10,0x04,0x00,0x00,0x00,0x00};
    unsigned char right_knee_lock[COMMAND_SIZE] = {0x53,0x10,0x10,0x14,0x00,0x00,0x00,0x00};
    unsigned char left_ankle_lock[COMMAND_SIZE] = {0x53,0x10,0x10,0x07,0x00,0x00,0x00,0x00};
    unsigned char right_ankle_lock[COMMAND_SIZE] = {0x53,0x10,0x10,0x17,0x00,0x00,0x00,0x00};
    unsigned char left_hip_lock[COMMAND_SIZE] = {0x53,0x10,0x10,0x02,0x00,0x00,0x00,0x00};
    unsigned char right_hip_lock[COMMAND_SIZE] = {0x53,0x10,0x10,0x12,0x00,0x00,0x00,0x00};   
	//unsigned char imu_Pitch[10]= {0x95,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	unsigned char clear_message[COMMAND_SIZE] = {0X53,0X13,0X33,0X00,0x00,0x00,0x00,0x00};

    unsigned char squat[COMMAND_SIZE] = {0x53,0x13,0x35,0x01,0x00,0x00,0x00,0x00};
    unsigned char stand[COMMAND_SIZE] = {0x53,0x13,0x35,0x02,0x00,0x00,0x00,0x00};

	//unsigned char stop_smooth[COMMAND_SIZE] = {0x53,0x13,0x11,0x02,0x00,0x00,0x00,0x00};
    unsigned char get_pos[COMMAND_SIZE] = {0x53,0x13,0x13,0x00,0x00,0x00,0x00,0x34};
    unsigned char clear_reset[COMMAND_SIZE] = {0x53,0x13,0x33,0x00,0x00,0x00,0x00,0x00};
   
    const int TIMER_SPAN_RATE_ = 200;
    
    const int ODOM_TIMER_SPAN_RATE_ = 30;

    const std::string BASE_FOOT_PRINT_;
    const std::string ODOM_FRAME_;
    
    Encoder ENCODER_{};
    char message_odom_[8];

    ros::NodeHandle nh_;
    ros::Publisher wheel_status_pub;
    ros::Publisher odom_raw_pub;
	
    tf::TransformBroadcaster broad_caster;

    ros::Subscriber cmd_vel_sub;
    ros::Subscriber joy_vel_sub;


    ros::Timer read_timer_;
    ros::Timer send_timer_;
    ros::Timer odom_publish_timer_;
    
    ros::Time cmd_vel_watch_{};
    ros::Time encoder_pre{}, encoder_after{}, encoder_stop{};


    Base_model BASE_MODEL_{};
    NaviSerialManager *serialManager;

    double linear_velocity_{}, angular_velocity_{};
    double global_x{0.0}, global_y{0.0}, global_theta{0.0};

    bool joy_vel_received_{};
    bool cmd_vel_received_{};
    bool ThreadRegistered_;
    bool right_updated{},left_updated{};
    bool publish_tf_{};

    unsigned char xor_msgs(unsigned char* msg);
    void init_send_msgs();
    void sendtimerCallback(const ros::TimerEvent & e);
    void readtimerCallback( const ros::TimerEvent & e);
    void odom_publish_timer_callback(const ros::TimerEvent & e);
    int parsingMsg();
    void odom_parsing();

    void joy_velCallback(const geometry_msgs::TwistConstPtr &msg);
    void cmd_velCallback(const geometry_msgs::TwistConstPtr &msg);
    //void SquatThread(double rate);
    
    
    //double angular_a[800],angular_k[800],angular_h[800];
    //double angle_a[800],angle_k[800],angle_h[800];

    std::mutex imu_mutex_,velocity_mutex_,command_mutex_;
    std::mutex left_mutex_,right_mutex_;
    bool send_imu_{};
public:
    BaseController(std::string serial_addr, unsigned int baudrate, std::string base_foot_print, std::string odom_frame,
                   std::string serial_addr1,std::string serial_addr2,bool publish_tf = false);
    ~BaseController();
    //std_msgs::UInt32 last_depth;
    //std_msgs::UInt32 latest_depth;
    //long double dp1,dp_change;
    //long double depth_value;
    void sendVelocity();
    //void sendForceSensor();
    //void send_joint_control(int i_);
    void sendCommand(double parameter = 0.0);
    void passCommand(Command user_command);
    void setBaseModel(const std::string &param_addr);
    //void squat_planning();
    //void joint_lock();
    //void SquatController(double rate);
    //bool NaviMode = true;
    //bool ControlMode = false;
    
    //void DropPreventionCallback (const sensor_msgs::Image & image);
	//void MultiImuCallback (const sensor_msgs::Imu & robot_Pitch);
};
#endif
