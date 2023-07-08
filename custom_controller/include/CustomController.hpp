#ifndef CUSTOMCONTROLLER
#define CUSTOMCONTROLLER

#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>

class CustomController
{
    public:

        //controller command pose
        float m_XPos;
        float m_YPos;
        float m_ZPos;
        float m_Yaw;
        float m_Pitch;
        float m_Roll;
        bool m_HasTransform;

    private:

        //initialization status
        bool m_IsInitialized;

        //frame
        std::string m_OdomFrame;
        std::string m_ControllerFrame;
        
        //pose offset
        tf2::Quaternion m_InitialRotation;
        tf2::Vector3 m_InitialTranslation;

        //relative pose change
        tf2::Quaternion m_RelativeRotation;
        tf2::Vector3 m_RelativeTranslation;

        //tf infomation
        tf2_ros::Buffer m_tfBuffer;
        tf2_ros::TransformListener m_tfListener;


    public:

        CustomController(std::string _OdomFrame, std::string _ControllerFrame);
        void Update();
        void UpdatePose();
        void UpdateCommand();
    
};





#endif