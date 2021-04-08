//#include "ros/ros.h"
//#include "ros/package.h"

#include "RobotController/basecontrol.h"
using namespace JOYTELEOP;
#include "stdio.h"
BaseController::BaseController(std::string serial_addr, unsigned int baudrate, std::string base_foot_print, std::string odom_frame,std::string serial_addr1,std::string serial_addr2,bool publish_tf)
{   
    
    serialManager = new NaviSerialManager(serial_addr, baudrate,8);
    if(serialManager -> openSerial())
    {
        serialManager -> registerAutoReadThread(TIMER_SPAN_RATE_);
        wheel_status_pub = nh_.advertise<RobotController::WheelStatus>("/wheel_status", 100);
        odom_raw_pub = nh_.advertise<nav_msgs::Odometry>("/odom_raw",100);
        cmd_vel_sub = nh_.subscribe("/cmd_vel", 100, &BaseController::cmd_velCallback, this);
        joy_vel_sub = nh_.subscribe("/joy_vel", 100, &BaseController::joy_velCallback, this);


        send_timer_ = nh_.createTimer(ros::Duration(1.0 / TIMER_SPAN_RATE_),&BaseController::sendtimerCallback,this);
        send_timer_.start();
        read_timer_ = nh_.createTimer(ros::Duration(4.0/TIMER_SPAN_RATE_),&BaseController::readtimerCallback,this);
        read_timer_.start();
        odom_publish_timer_ = nh_.createTimer(ros::Duration(1.0/ODOM_TIMER_SPAN_RATE_), &BaseController::odom_publish_timer_callback, this);
        odom_publish_timer_.start();
        ROS_INFO_STREAM("BASE READY");
    }
    else
    ROS_ERROR_STREAM("Can't open "<<"SERIAL "<<serial_addr<<std::endl);
}       

BaseController::~BaseController()
{
    delete serialManager;
}

void BaseController::joy_velCallback(const geometry_msgs::TwistConstPtr &msg)
{
    if(cmd_vel_received_)
		cmd_vel_received_=false;
    //Cmd_vel user_cmd_vel;
    
    double linear_velocity{msg->linear.x};
    double angular_velocity{msg->angular.z};
    double right_vel, left_vel;

    if(linear_velocity == 0)
    {
        right_vel = angular_velocity * BASE_MODEL_.Wheel_Base/2000.0; //m
        left_vel  = -right_vel;
    }
    else if(angular_velocity == 0)
        right_vel = left_vel = linear_velocity;
    else
    {
        right_vel  = linear_velocity + angular_velocity*BASE_MODEL_.Wheel_Base/2000.0; //m
        left_vel   = linear_velocity - angular_velocity*BASE_MODEL_.Wheel_Base/2000.0; //m
    }

    joy_vel_received_ = linear_velocity!=0||angular_velocity!=0 ;
    //when there is no-zero cmd velocity, do not send zero joy velocity
    if(!cmd_vel_received_||joy_vel_received_)
    {
        velocity_mutex_.lock();
        user_cmd_vel_.cmd_right_wheel = right_vel*60.0/(M_PI * BASE_MODEL_.Wheel_Diameter/1000.0);
        user_cmd_vel_.cmd_left_wheel  = left_vel *60.0/(M_PI * BASE_MODEL_.Wheel_Diameter/1000.0);
        velocity_mutex_.unlock();
    }
    //cmd_vel_received_ = linear_velocity!=0||angular_velocity!=0 ;
    //ROS_INFO_STREAM("GET INFORMATION FROM JOY AND SEND VELOCITY");
}

void BaseController::cmd_velCallback(const geometry_msgs::TwistConstPtr &msg)
{
    cmd_vel_watch_ = ros::Time::now();   
    if(joy_vel_received_)
        return;

    //Cmd_vel user_cmd_vel{};
    double linear_velocity{msg->linear.x};
    double angular_velocity{msg->angular.z};
    double right_vel{}, left_vel{};
    if(linear_velocity == 0)
    {
        right_vel = angular_velocity * BASE_MODEL_.Wheel_Base/2000.0; //m
        left_vel  = -right_vel;
    }
    else if(angular_velocity == 0)
        right_vel = left_vel = linear_velocity;
    else
    {
        right_vel  = linear_velocity + angular_velocity*BASE_MODEL_.Wheel_Base/2000.0; //m
        left_vel   = linear_velocity - angular_velocity*BASE_MODEL_.Wheel_Base/2000.0; //m
    }

    cmd_vel_received_ = linear_velocity!=0||angular_velocity!=0 ;

    velocity_mutex_.lock();
    user_cmd_vel_.cmd_right_wheel = right_vel*60.0/(M_PI * BASE_MODEL_.Wheel_Diameter/1000.0);
    user_cmd_vel_.cmd_left_wheel  = left_vel *60.0/(M_PI * BASE_MODEL_.Wheel_Diameter/1000.0);
    //sendVelocity(user_cmd_vel_);
    velocity_mutex_.unlock();
}

void BaseController::init_send_msgs()
{
    //get_pos[7]=xor_msgs(get_pos);
}

unsigned char BaseController::xor_msgs(unsigned char *msg)
{
    unsigned char check=msg[1];
    for(int i=2;i<7;i++)
        check=check ^ msg[i];
    return check;
}

void BaseController::sendVelocity()
{
    velocity_mutex_.lock();
   wheel_vel[5] = user_cmd_vel_.cmd_left_wheel;
   wheel_vel[6] = -user_cmd_vel_.cmd_right_wheel;
   velocity_mutex_.unlock();
   serialManager -> send(wheel_vel, COMMAND_SIZE);
   //std::cout << "sendVelocity" << std::endl;
}

void BaseController::setBaseModel(const std::string & param_addr)
{
    global_x -= BASE_MODEL_.Wheel_Center_X_Offset;
    global_y -= BASE_MODEL_.Wheel_Center_Y_Offset;
    YAML::Node doc = YAML::LoadFile(param_addr);
    try
    {
        BASE_MODEL_.Wheel_Diameter=doc["WheelDiameter"].as<double>();
        BASE_MODEL_.Wheel_Base = doc["WheelBase"].as<double>();
        BASE_MODEL_.Encoder_to_Distance = doc["EncoderToDistance"].as<double>();
        BASE_MODEL_.Wheel_Center_X_Offset = doc["WheelCenterXOffset"].as<double>();
        BASE_MODEL_.Wheel_Center_Y_Offset = doc["WheelCenterYOffset"].as<double>();
    }
    catch (YAML::InvalidScalar)
    {
        ROS_ERROR("tagParam.yaml is invalid.");
    }
    global_x+=BASE_MODEL_.Wheel_Center_X_Offset;
    global_y+=BASE_MODEL_.Wheel_Center_Y_Offset;
}

void BaseController::sendtimerCallback(const ros::TimerEvent &e)
{
    static int encoder_counter=1;

    //sendcommand
	if(encoder_counter%4 == 1)//100hz
    {
        //serialManager -> send(get_JointEncoder,COMMAND_SIZE);
        //LeftForceSensor -> send(get_force,4);
        //RightForceSensor -> send(get_force,4);
    }
    if(encoder_counter%4 == 3)
    {

    }
	if(encoder_counter%4==0)   //50hz
        serialManager -> send(get_pos,COMMAND_SIZE);
    if(encoder_counter % 8 ==2) //25hz
        sendCommand();
    if(encoder_counter % 8 == 6)//25hz
        sendVelocity();
	/*if(encoder_counter == 236&&send_imu_)
    {
		//15 hz
	    imu_mutex_.lock();
        serialManager -> send(imu_Pitch,10);
        imu_mutex_.unlock();
    }*/
	encoder_counter++;
	encoder_counter = encoder_counter == TIMER_SPAN_RATE_*2+1 ? 1 : encoder_counter;
}

void BaseController::readtimerCallback(const ros::TimerEvent &e)
{
    NaviSerialManager::ReadResult self_results{serialManager->getReadResult()};
    encoder_pre = encoder_after;
    if(self_results.read_bytes>=COMMAND_SIZE)
    {
        int k = 8;
        for (int i = 0; i < self_results.read_bytes; i += k)
        {
            if(self_results.read_result[i] == COMMAND_HEAD)
            {
                k = 8;
                memcpy(message_odom_,&self_results.read_result[i],COMMAND_SIZE);
                parsingMsg();
            }
            else if(self_results.read_result[i] == ENCODER_HEAD)
            {
                k = 12;
                //memcpy(message_encoder_,&self_results.read_result[i],ENCODER_SIZE);
                //parsingEncoder();
            }
            else
            {
                ROS_WARN_STREAM(" THE DATA OF ODOM/ENCODER IS ERROR");  
            }
        }
          
        if(right_updated&&left_updated)
        {
            ENCODER_.interval=(encoder_after-encoder_pre).toSec();
            odom_parsing();
            right_updated=false;
            left_updated=false;
        }
    }
    else
        memset(message_odom_, 0, COMMAND_SIZE);
        //memset(message_encoder_,0,ENCODER_SIZE);
    //publish the encoder when have.
        if(ENCODER_.interval!=0)
        {
            if(ENCODER_.interval>20.0/TIMER_SPAN_RATE_||ENCODER_.interval<0.0)
            {
                if(!ENCODER_.encoderWrong)
                 encoder_stop=ros::Time::now();
                ENCODER_.encoderWrong=true;
                ROS_ERROR_STREAM("Encoder once passed 5 frames");
            }
            else
            {   
                if(ENCODER_.encoderWrong)
                {
                    if((encoder_pre-encoder_stop).toSec()>=1.0)
                        ENCODER_.encoderWrong=false;
                }
            }
            RobotController::WheelStatus wheelStatus{};
            wheelStatus.right_encoder=ENCODER_.right_encoder;
            wheelStatus.left_encoder = ENCODER_.left_encoder;
            wheelStatus.encoderWrong = ENCODER_.encoderWrong;
            wheelStatus.interval = ENCODER_.interval;
            wheel_status_pub.publish(wheelStatus);
        }
}

void BaseController::sendCommand( double parameter)
{
    command_mutex_.lock();
    switch (user_command_)
    {
		case IMU:
			send_imu_ = !send_imu_;
        case STOP:
            break;
        case GET_ENCODER:
            //serialManager -> send(get_encoder,COMMAND_SIZE);
            break;
        case GET_POSE:
            //serialManager->send(get_pos,COMMAND_SIZE);
            break;
        case HIPLOCK:
            break;
        case HIPCUT:
            break;
        case ANKLELOCK:
            serialManager -> send(left_ankle_lock,COMMAND_SIZE);
            usleep(5000);
            serialManager -> send(right_ankle_lock,COMMAND_SIZE);
            break;
        case ANKLECUT:
            //serialManager -> send(left_ankle_cut,COMMAND_SIZE);
			//std::cout << "anklecut 1" << std::endl;	
            usleep(500000);
            serialManager -> send(clear_message,COMMAND_SIZE);
            break;
        case KNEELOCK:
            break;
        case KNEECUT:
            serialManager -> send(left_knee_cut,COMMAND_SIZE);
			std::cout << "kneecut 1" << std::endl;
            usleep(500000);
            serialManager -> send(right_knee_cut,COMMAND_SIZE);
            break;
        case RESET:
			std::cout<<"reset here"<<std::endl;
            serialManager -> send(joint_reset,COMMAND_SIZE);
            break;
        case CLEARRESET:
            serialManager -> send(clear_reset,COMMAND_SIZE);
            break;
        case INIT:
            serialManager -> send(joint_init,COMMAND_SIZE);
            break;
        case SQUAT:
            serialManager -> send(squat,COMMAND_SIZE);
            break;
        case STAND:
            serialManager -> send(stand,COMMAND_SIZE);
            break;
        default:
            break;
    }
    user_command_ = Command::DEFAULT;
    command_mutex_.unlock();
}

void BaseController::passCommand(Command user_command)
{
    command_mutex_.lock();
    user_command_ = user_command;
    command_mutex_.unlock();
}

int BaseController::parsingMsg()
{
    if(0x35!=message_odom_[0])
    {
        memset(message_odom_,0,COMMAND_SIZE);
        return -1;
    }
    else
    {
        switch (message_odom_[1])
        {
            case 0x31:
                /*preserved*/
                break;
            case 0x21:
                /*poistion of right wheel*/
                if(0x13==message_odom_[2])
                {
                    //right encodisk parsing
                    char* pchar = (char*)&ENCODER_.right_encoder;
                    *(pchar+3) = message_odom_[3];
                    *(pchar+2) = message_odom_[4];
                    *(pchar+1) = message_odom_[5];
                    *(pchar+0) = message_odom_[6];

                    if (std::abs(ENCODER_.right_encoder) > INT_MAX - 1000)
                        ENCODER_.right_encoder = 0;

                    //make sure the consistency of left and right
                    right_updated= true;
                    encoder_after = ros::Time::now();
                }
                break;
            case 0x11:
                /*position of left wheel*/
                if(0x13==message_odom_[2])
                {
                    char* pchar = (char*)&ENCODER_.left_encoder;
                    *(pchar+3) = message_odom_[3];
                    *(pchar+2) = message_odom_[4];
                    *(pchar+1) = message_odom_[5];
                    *(pchar+0) = message_odom_[6];

                    //按旧小车，向前进时，左轮码盘为负，要乘负一
                    ENCODER_.left_encoder *=-1;

                    if (std::abs(ENCODER_.left_encoder) > INT_MAX - 1000)
                        ENCODER_.left_encoder = 0;
                    //make sure the consistency of left and right
                    left_updated=true;
                    encoder_after = ros::Time::now();
                }
                break;
			default:
                break;
        }
    }
    return 0;
}

void BaseController::odom_parsing()
{
    static int right_encoder_pre{ENCODER_.right_encoder},left_encoder_pre{ENCODER_.left_encoder};
    static ros::Time last_time{ros::Time::now()};

    int right_delta{ENCODER_.right_encoder - right_encoder_pre};
    int left_delta{ENCODER_.left_encoder - left_encoder_pre};


    double dt = (ros::Time::now()-last_time).toSec();
    double right_distance{},left_distance{};
    double theta{};

    // the robot is not moving
    if(abs(left_delta)<2&&abs(right_delta)<2)
    {
        linear_velocity_=0;
        angular_velocity_=0;
        //global position don't change

        //update value
        right_encoder_pre = ENCODER_.right_encoder;
        left_encoder_pre = ENCODER_.left_encoder;
        last_time = ros::Time::now();
        return;
    }

    //in case of wrong data poisoning;
    //Noted, this is because  value bigger than the int limit;
    if(abs(left_delta)>1000||abs(right_delta)>1000)
    {
        //we hope to keep the same value of last time
        //so just update value and return;

        right_encoder_pre = ENCODER_.right_encoder;
        left_encoder_pre = ENCODER_.left_encoder;
        last_time = ros::Time::now();
        return ;
    }

    //in case of EMERGENCY button pressed
    if(ENCODER_.right_encoder==0&&ENCODER_.left_encoder==0&&right_encoder_pre!=0&&left_encoder_pre!=0)
    {
        //we hope to keep the same value of last time
        //so just update value and return;

        right_encoder_pre = ENCODER_.right_encoder;
        left_encoder_pre = ENCODER_.left_encoder;
        last_time = ros::Time::now();
        return ;
    }


    right_distance = M_PI * BASE_MODEL_.Wheel_Diameter * right_delta / BASE_MODEL_.Encoder_to_Distance;//mm
    left_distance  = M_PI * BASE_MODEL_.Wheel_Diameter * left_delta / BASE_MODEL_.Encoder_to_Distance;//mm
    theta = (right_distance-left_distance)/BASE_MODEL_.Wheel_Base;
    if(left_delta==right_delta)
    {
        //moving forward
        double vertical_robot{((right_distance+left_distance)/2.0)};//mm
        vertical_robot /= 1000.0; //mm to m;
        double horizontal_robot{};//m

        global_x += vertical_robot*cos(global_theta) - horizontal_robot*sin(global_theta);//m
        global_y += horizontal_robot*cos(global_theta) + vertical_robot*sin(global_theta);//m
        global_theta +=theta;

        linear_velocity_ = (right_distance+left_distance)/(2000.0*dt);
        angular_velocity_ = theta/dt;
    }
    else
    {
        //turing round
        double turning_radius;
        turning_radius =(BASE_MODEL_.Wheel_Base*(right_distance+left_distance))/(2.0*(right_distance-left_distance));//m

        double vertical_robot{turning_radius*sin(theta)};//mm
        double horizontal_robot{turning_radius*(1-cos(theta))};//mm

        vertical_robot /= 1000.0;//m
        horizontal_robot /= 1000.0;//m

        global_x += vertical_robot*cos(global_theta) - horizontal_robot*sin(global_theta);//m
        global_y += horizontal_robot*cos(global_theta) + vertical_robot*sin(global_theta);//m
        global_theta +=theta;

        linear_velocity_ = (right_distance+left_distance)/(2000.0*dt);
        angular_velocity_ = theta/dt;
    }

    //update value
    right_encoder_pre = ENCODER_.right_encoder;
    left_encoder_pre = ENCODER_.left_encoder;
    last_time = ros::Time::now();
}

void BaseController::odom_publish_timer_callback(const ros::TimerEvent &e)
{
    nav_msgs::Odometry odom;
    geometry_msgs::TransformStamped odom_trans;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(global_theta);

    odom.header.stamp = odom_trans.header.stamp = ros::Time::now();
    odom.header.frame_id = odom_trans.header.frame_id = ODOM_FRAME_;
    odom.child_frame_id = odom_trans.child_frame_id = BASE_FOOT_PRINT_;

    odom.pose.pose.position.x = odom_trans.transform.translation.x =global_x;
    odom.pose.pose.position.y = odom_trans.transform.translation.y =global_y;
    odom.pose.pose.position.z = odom_trans.transform.translation.z =0.0;
    odom.pose.pose.orientation = odom_trans.transform.rotation = odom_quat;

    odom.pose.covariance=	{1e-3,0,0,0,0,0,
                              0,1e-3,0,0,0,0,
                              0,0,1e6,0,0,0,
                              0,0,0,1e6,0,0,
                              0,0,0,0,1e6,0,
                              0,0,0,0,0,1e3};

    odom.twist.twist.linear.x = linear_velocity_;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = angular_velocity_;

    odom.twist.covariance={1e-3,0,0,0,0,0,
                           0,1e-3,0,0,0,0,
                           0,0,1e6,0,0,0,
                           0,0,0,1e6,0,0,
                           0,0,0,0,1e6,0,
                           0,0,0,0,0,1e3};

    odom_raw_pub.publish(odom);

    if(publish_tf_)
        broad_caster.sendTransform(odom_trans);
}
