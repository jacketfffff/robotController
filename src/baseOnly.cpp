
#include <ros/ros.h>
#include "RobotController/basecontrol.h"
#include "ros/package.h"
#include "basecontrol.cpp"
#include "RobotController/JoyTeleop.h"

using namespace JOYTELEOP;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "base_Only");
    ros::NodeHandle nh_("~");
    const std::string parameter_addr{ros::package::getPath("basecontrol")+"/config/basemodel.yaml"};//添加参数文件
	int Hz_count = 0;
    double max_linear_velocity, max_angular_velocity;
    std::string base_foot_print,odom_frame,map_frame,serial_addr,serial_addr1,serial_addr2;
    bool publish_tf;

    nh_.param("publish_tf", publish_tf, (bool)false);
    nh_.param("base_foot_print", base_foot_print, (std::string)"base_link");
    nh_.param("odom_frame", odom_frame, (std::string)"odom");
    nh_.param("serial_addr", serial_addr, (std::string)"/dev/ttyUSB0");
    nh_.param("serial_addr1", serial_addr1, (std::string)"/dev/ttyUSB0");
    nh_.param("serial_addr2", serial_addr2, (std::string)"/dev/ttyUSB0");

    JoyTeleop joyTeleop("/joy",true,0.3,0.6);
    BaseController baseController(serial_addr, B115200, base_foot_print, odom_frame, serial_addr1,serial_addr2,publish_tf);
   

    baseController.setBaseModel(parameter_addr);

    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::Rate loop_rate(30);

    while(ros::ok)
    {
        switch(joyTeleop.getControlTrigger())
        {
            case Reset:
                baseController.passCommand(baseController.RESET);
                std::cout << "Reset" << std::endl;
                break;//B 关节回零
            case Init:
                baseController.passCommand(baseController.INIT);
                std::cout << "Init" << std::endl;
                break;// X 关节初始化
            case ClearReset:
                baseController.passCommand(baseController.CLEARRESET);
                std::cout << "ClearReset" << std::endl;
                break;// Y 清除关节回零
            case Stand:
                std::cout << "Stand" << std::endl;
                baseController.passCommand(baseController.STAND);
                break;//Up
            case Squat: 
                std::cout << "Squat" << std::endl;
                baseController.passCommand(baseController.SQUAT);
                break;//DOWN
            case Stop:
                std::cout << "Stop" << std::endl;
                baseController.passCommand(baseController.STOP);
                break;//select
            case Kneecut:
                std::cout << "Kneecut" << std::endl;
                baseController.passCommand(baseController.KNEECUT);
                break;//right + L1
            case KneeLock:
                baseController.passCommand(baseController.KNEELOCK);
                std::cout << "KneeLock" << std::endl;
                break;//left + L1
            case Anklecut:
                std::cout << "Anklecut" << std::endl;
                baseController.passCommand(baseController.ANKLECUT);
                break;//right
            case AnkleLock:
                baseController.passCommand(baseController.ANKLELOCK);
                std::cout << "AnkleLock" << std::endl;
                break;//left
            case Hipcut:
                baseController.passCommand(baseController.HIPCUT);
                std::cout << "Hipcut" << std::endl;
                break;//right + L2
            case HipLock:
                baseController.passCommand(baseController.HIPLOCK);
                std::cout << "HipLock" << std::endl;
                break;//left + L2
            case Imustart:
                std::cout<<"Imu status changed" <<std::endl;
                baseController.passCommand(baseController.IMU);
                break;//start
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
