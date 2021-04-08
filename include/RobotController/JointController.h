class JointController{
public:
	struct Joint_control {
        int left_ankle_vel;
        int left_knee_vel;
        int left_hip_vel;
        int right_ankle_vel;
        int right_knee_vel;
        int right_hip_vel;
        int left_ankle_angular;
        int left_knee_angular;
        int left_hip_angular;
        int right_ankle_angular;
        int right_knee_angular;
        int right_hip_angular;
    };
private:
	unsigned char get_JointEncoder[COMMAND_SIZE] = {0X93,0X13,0X00,0X00,0X00,0X00,0X00,0X00};
	unsigned char get_force[4] = {0x49, 0xAA, 0x0D, 0x0A};
	double Rightforcesensor_[6];
	double Leftforcesensor_[6];

	const float FORCE_TRANSFORM_ =  0.0390625 * 9.8;
	const float MOMENT_TRANSFORM_ = 0.0009765625 * 9.8;

	char message_encoder_[12];
	char leftforcesensor_[12];
	char rightforcesensor_[12];

	ros::NodeHandle nh_;
	ros::Publisher left_forcesensor_pub;
	ros::Publisher right_forcesensor_pub;
	ros::Publisher right_encoder_pub;
	ros::Publisher left_encoder_pub;
	ros::Publisher shuiping_depth;

	ros::Subscriber DropPrevention_Sub;	
	ros::Subscriber multi_imu;

	ros::Timer left_forcesensor_timer_;
	ros::Timer right_forcesensor_timer_;

	ros::Time drop_prevention_time_1{};
	ros::Time drop_prevention_time_2{};
	ros::Time drop_prevention_time_3{};
	ros::Time drop_prevention_time_4{};

	NaviSerialManager *LeftForceSensor;
	NaviSerialManager *RightForceSensor;

	void leftforcesensorCallback( const ros::TimerEvent & e);
	void rightforcesensorCallback(const ros::TimerEvent & e);
	int parsingEncoder();
};
