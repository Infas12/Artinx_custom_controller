#include "serial/serial.h"
#include "Math.hpp"
#include <iostream>
#include "TransmitterJS.hpp"
#include "CustomController.hpp"

int main(int argc, char **argv)
{
    // Initialize ros stuff
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle nh;
    ros::Rate loopRate(100);

    // Initialize Serial
    serial::Serial testSerial("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(1000));  
    TransmitterJS::Instance()->m_p_serialPort = &testSerial;
 
    CustomController controller("camera_odom_frame","camera_pose_frame");

    while (ros::ok())
    {
        
        Time::tick();

        controller.Update();

        if(Time::getTick()%4 == 0){ //send every 40 ms (25hz)
            CustomControllerPacketHandler::Instance()->m_XPos = controller.m_XPos;
            CustomControllerPacketHandler::Instance()->m_YPos = controller.m_YPos;
            CustomControllerPacketHandler::Instance()->m_ZPos = controller.m_ZPos;
            CustomControllerPacketHandler::Instance()->m_Roll = controller.m_Roll;
            CustomControllerPacketHandler::Instance()->m_Pitch= controller.m_Pitch;
            CustomControllerPacketHandler::Instance()->m_Yaw  = controller.m_Yaw;
            CustomControllerPacketHandler::Instance()->m_HasTransform = static_cast<uint8_t>(controller.m_HasTransform);
            CustomControllerPacketHandler::Instance()->SendPacket();
        }


        TransmitterJS::Instance()->Update();

        // loop control
        ros::spinOnce();
        loopRate.sleep();

    }
}
