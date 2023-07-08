#include <iostream>
#include "Packet/HostPacket.hpp"

HostPacketManager::HostPacketManager() : PacketManager(1)
{
    m_pChannel = &m_StreamChannel;
}

void HostPacketManager::Init()
{
    PacketManager::Init();
}

void HostPacket::SendPacket()
{

    if (!m_p_serialPort->isOpen() || m_p_serialPort == nullptr)
    {
        return;
    }

    m_p_serialPort->write(m_Buffer, m_DataLen + 8);
}

static uint8_t buffer[64];
void HostPacketManager::Update()
{
    if (m_p_serialPort == nullptr)
    {
        return;
    }

    // collect data from serial port.
    uint8_t length = m_p_serialPort->available();
    m_p_serialPort->read(buffer, length);
    Enqueue(0, buffer, length);
    PacketManager::Update();
}

void EncoderPacket::Send(float _ECD0, float _ECD1, float _ECD2, float _ECD3)
{
    m_Encoder0 = _ECD0;
    m_Encoder1 = _ECD1;
    m_Encoder2 = _ECD2;
    m_Encoder3 = _ECD3;

    SerializePacket(m_OutputStream);
    HostPacket::SendPacket();
}

void EncoderPacket::OnPacketReceived()
{
    SerializePacket(m_InputStream);
    std::cout << m_Encoder0 << " " << m_Encoder1 << " " << m_Encoder2 << " " << m_Encoder3 << std::endl;
}

template <typename Stream>
void EncoderPacket::SerializePacket(Stream &stream)
{
    m_DataLen = 6;

    SerializeHeader(stream);

    stream.SerializeFloat(m_Encoder0, -3.14159, 0.0001f);
    stream.SerializeFloat(m_Encoder1, -3.14159, 0.0001f);
    stream.SerializeFloat(m_Encoder2, -3.14159, 0.0001f);
    stream.SerializeFloat(m_Encoder3, -3.14159, 0.0001f);

    SerializeCrc16(stream);
}

void MotorSetPacket::Send(float _set0, float _set1, float _set2, float _set3)
{
    m_posSet0 = _set0;
    m_posSet1 = _set1;
    m_posSet2 = _set2;
    m_posSet3 = _set3;

    SerializePacket(m_OutputStream);
    HostPacket::SendPacket();
}

void MotorSetPacket::OnPacketReceived()
{
    SerializePacket(m_InputStream);
}

template <typename Stream>
void MotorSetPacket::SerializePacket(Stream &stream)
{
    m_DataLen = 8;

    SerializeHeader(stream);

    stream.SerializeFloat(m_posSet0, -3.14159, 0.0001f);
    stream.SerializeFloat(m_posSet1, -3.14159, 0.0001f);
    stream.SerializeFloat(m_posSet2, -3.14159, 0.0001f);
    stream.SerializeFloat(m_posSet3, -3.14159, 0.0001f);

    SerializeCrc16(stream);
}

void VelcmdPacket::Send(float _Vx, float _Vy, float _Vw)
{
    m_Vx = _Vx;
    m_Vy = _Vy;
    m_Vw = _Vw;

    SerializePacket(m_OutputStream);
    HostPacket::SendPacket();
}

void VelcmdPacket::OnPacketReceived()
{
    SerializePacket(m_InputStream);
    std::cout << m_Vx << " " << m_Vy << " " << m_Vw << std::endl;
}

template <typename Stream>
void VelcmdPacket::SerializePacket(Stream &stream)
{
    m_DataLen = 12;

    SerializeHeader(stream);

    stream.SerializeFloat(m_Vx);
    stream.SerializeFloat(m_Vy);
    stream.SerializeFloat(m_Vw);

    SerializeCrc16(stream);
}


void PosecmdPacket::Send(float _x, float _y, float _z, float _yaw, float _pitch, float _roll)
{
    m_x = _x;
    m_y = _y;
    m_z = _z;
    m_yaw = _yaw;
    m_pitch = _pitch;
    m_roll = _roll;

    SerializePacket(m_OutputStream);
    HostPacket::SendPacket();
}

void PosecmdPacket::OnPacketReceived()
{
    SerializePacket(m_InputStream);
}

template <typename Stream>
void PosecmdPacket::SerializePacket(Stream &stream)
{
    m_DataLen = 18;

    SerializeHeader(stream);

    stream.SerializeFloat(m_roll,-3.14f,0.0001);
    stream.SerializeFloat(m_pitch,-3.14f,0.0001);
    stream.SerializeFloat(m_yaw,-3.14f,0.0001);
    stream.SerializeFloat(m_x);
    stream.SerializeFloat(m_y);
    stream.SerializeFloat(m_z);



    SerializeCrc16(stream);
}










