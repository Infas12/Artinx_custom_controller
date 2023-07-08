#include "CustomController.hpp"

#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <iostream>
#include "Math.hpp"


CustomController::CustomController(std::string _OdomFrame, std::string _ControllerFrame):
    m_OdomFrame(_OdomFrame),
    m_ControllerFrame(_ControllerFrame),
    m_IsInitialized(false),
    m_HasTransform(false),
    m_tfListener(m_tfBuffer),
    m_InitialRotation(0,0,0,1),
    m_RelativeRotation(0,0,0,1),
    m_InitialTranslation(0,0,0),
    m_RelativeTranslation(0,0,0)
{}

void CustomController::Update()
{
    UpdatePose();
    UpdateCommand();
}


static double lastUpdateTick = 0;
void CustomController::UpdatePose()
{
    //pose in odom frame
    tf2::Quaternion _quat_odom_frame;
    tf2::Vector3 _trans_odom_frame;

    geometry_msgs::TransformStamped _transformStamped;

    //Read raw tf infomation
    try
    {
        _transformStamped = m_tfBuffer.lookupTransform("camera_odom_frame", "camera_pose_frame", ros::Time(0));
        _quat_odom_frame.setX(_transformStamped.transform.rotation.x);
        _quat_odom_frame.setY(_transformStamped.transform.rotation.y);
        _quat_odom_frame.setZ(_transformStamped.transform.rotation.z);
        _quat_odom_frame.setW(_transformStamped.transform.rotation.w);
        _trans_odom_frame.setX(_transformStamped.transform.translation.x);    
        _trans_odom_frame.setY(_transformStamped.transform.translation.y);
        _trans_odom_frame.setZ(_transformStamped.transform.translation.z);


        //to be fixed
        if(_transformStamped.header.stamp.toSec() == lastUpdateTick){
            m_HasTransform = false;
        }else{
            m_HasTransform = true;
        }

        lastUpdateTick = _transformStamped.header.stamp.toSec();
        
    }
    catch(tf2::TransformException &ex)//if no tf published then exit
    {
        ROS_ERROR("%s", ex.what());
        m_HasTransform = false;
        return;
    }

    //get initial pose
    if(!m_IsInitialized)
    {
        m_InitialRotation = _quat_odom_frame;
        m_InitialTranslation = _trans_odom_frame;
        m_IsInitialized = true;
    }

    //move relative to initial frame 
    m_RelativeRotation = m_InitialRotation.inverse() * _quat_odom_frame;
    m_RelativeTranslation = tf2::quatRotate(m_InitialRotation.inverse(),_trans_odom_frame - m_InitialTranslation);
    
}

void CustomController::UpdateCommand()
{

    //from meter to millimeter
    float _rawX, _rawY, _rawZ;
    _rawX = m_RelativeTranslation.getX() * 1000;
    _rawY = m_RelativeTranslation.getY() * 1000;
    _rawZ = m_RelativeTranslation.getZ() * 1000;

    //calculate euler angle from quaternion.
    //The rotation sequence is 3-2-1 (Z-Y-X), which is same as the rotation sequence of the end effecter.
    tf2Scalar _rawYaw, _rawPitch, _rawRoll = 0.0;
    tf2::Matrix3x3 _mat(m_RelativeRotation);
    _mat.getEulerYPR(_rawYaw,_rawPitch,_rawRoll);

    //add offset to pitch
    _rawPitch = _rawPitch + 3.14159f/2.0f;
    _rawPitch = Math::LoopFloatConstrain(_rawPitch,-3.1415926,3.1415926);

    //pitch constraint
    _rawPitch = Math::FloatConstrain(_rawPitch,1.6,2.616);

    //position constraint
    _rawX = Math::FloatConstrain(_rawX,-314,386);
    _rawY = Math::FloatConstrain(_rawY,-500,500);
    _rawZ = Math::FloatConstrain(_rawZ,-228,228);

    //Update
    m_XPos  = _rawX;
    m_YPos  = _rawY;
    m_ZPos  = _rawZ;
    m_Roll  = _rawRoll;
    m_Pitch = _rawPitch;
    m_Yaw   = _rawYaw;

}