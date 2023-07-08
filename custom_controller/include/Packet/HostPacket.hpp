#ifndef HOSTPACKET_HPP
#define HOSTPACKET_HPP

#include "Packet.hpp"
#include "serial/serial.h"

class HostPacketManager : public PacketManager
{
private:
    StreamChannel m_StreamChannel;

public:
    HostPacketManager();

    serial::Serial *m_p_serialPort; // must be set before Packet registration.

    virtual void Init();
    virtual void Update();
    static HostPacketManager *Instance()
    {
        static HostPacketManager instance;
        return &instance;
    }
};

class HostPacket : public Packet
{
private:
    serial::Serial *m_p_serialPort;

public:
    HostPacket(uint8_t _id) : Packet(_id) {}

    virtual void Registration()
    {
        HostPacketManager::Instance()->PacketHandlerRegistration(0, m_ProtocolId, this);
        m_p_serialPort = HostPacketManager::Instance()->m_p_serialPort;
    }

    virtual void SendPacket();
};

class EncoderPacket : public HostPacket
{
private:
    float m_Encoder0;
    float m_Encoder1;
    float m_Encoder2;
    float m_Encoder3;

public:
    EncoderPacket(uint8_t _id) : HostPacket(_id) {}
    void Send(float _ECD0, float _ECD1, float _ECD2, float _ECD3);
    virtual void OnPacketReceived();
    template <typename Stream>
    void SerializePacket(Stream &stream);
};

class MotorSetPacket : public HostPacket
{
public:
    float m_posSet0;
    float m_posSet1;
    float m_posSet2;
    float m_posSet3;

public:
    MotorSetPacket(uint8_t _id) : HostPacket(_id) {}
    void Send(float _set0, float _set1, float _set2, float _set3);
    virtual void OnPacketReceived();
    template <typename Stream>
    void SerializePacket(Stream &stream);
};

class VelcmdPacket : public HostPacket
{
private:
    float m_Vx;
    float m_Vy;
    float m_Vw;

public:
    VelcmdPacket(uint8_t _id) : HostPacket(_id) {}
    void Send(float _Vx, float _Vy, float _Vw);
    virtual void OnPacketReceived();
    template <typename Stream>
    void SerializePacket(Stream &stream);

    static VelcmdPacket *Instance()
    {
        static VelcmdPacket instance(0x80);
        return &instance;
    }
};

class PosecmdPacket : public HostPacket
{
private:
    float m_x;
    float m_y;
    float m_z;
    float m_yaw;
    float m_pitch;
    float m_roll;

public:
    PosecmdPacket(uint8_t _id) : HostPacket(_id) {}
    void Send(float _x, float _y, float _z, float _yaw, float _pitch, float _roll);
    virtual void OnPacketReceived();
    template <typename Stream>
    void SerializePacket(Stream &stream);

    static PosecmdPacket *Instance()
    {
        static PosecmdPacket instance(0x80);
        return &instance;
    }
};





#endif
