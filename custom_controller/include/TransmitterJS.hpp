#ifndef JUDGESYSTEM_HPP
#define JUDGESYSTEM_HPP

#include <cstring>
#include <stdint.h>
#include "Time.hpp"
#include "Packet/Stream.hpp"
#include "Crc.hpp"
#include "HashTable.hpp"

#include "Packet/MemoryStream.hpp"

#include "serial/serial.h"

#define JS_RX_QUEUE_LEN 128
#define JS_FRAME_BUFFER_LEN 150
#define JS_SEND_BUFFER_LEN 512


class TransmitterJS
{
public:
    enum JudgePacketType
    {
        JPT_InteractiveData = 0x0302,
        JPT_ClientCommand = 0x0304
    };

    struct JS_InteractiveData
    {
        uint8_t data[30];
    }; 

    typedef struct JS_ClientCommand
    {
        uint16_t key_value;
        uint16_t x_position:12;
        uint16_t mouse_left:4;
        uint16_t y_position:12;
        uint16_t mouse_right:4;
        uint16_t reserved;
    };

private:


    uint8_t m_RxQuene[JS_RX_QUEUE_LEN];
    uint8_t m_FrameBuffer[JS_FRAME_BUFFER_LEN];
    uint8_t m_CurrentFrame[JS_FRAME_BUFFER_LEN];
    uint8_t m_HeaderBuffer[5];
    uint8_t m_SendBuffer[JS_SEND_BUFFER_LEN];

    uint8_t m_RxQueueFront;
    uint8_t m_RxQueueRear;
    uint8_t m_FrameBufferLen;
    uint8_t m_HeaderBufferLen;
    uint16_t m_ExpectedLen;
    uint8_t m_SendSeq;
    uint16_t m_SendBufferLen;
    uint32_t m_headerperS;
    uint32_t m_hs;
    bool m_CheckingHeader;

    const static uint8_t Magic_Number;
    const static uint8_t Header_Length;
    const static uint32_t Flush_Tick;

    TransmitterJS() : m_RxQueueFront(0),
                    m_RxQueueRear(0),
                    m_FrameBufferLen(0),
                    m_HeaderBufferLen(0),
                    m_ExpectedLen(0),
                    m_SendSeq(0),
                    m_SendBufferLen(0),
                    m_headerperS(0),
                    m_hs(0),
                    m_CheckingHeader(false)
    {}

    void HandlePacket();
    void FlushBuffer();

public:
    static TransmitterJS* Instance()
    {
        static TransmitterJS instance;
        return &instance;
    }

    void Enqueue(uint8_t _data)
    {
        m_RxQuene[m_RxQueueRear] = _data;
        m_RxQueueRear = (m_RxQueueRear + 1) % JS_RX_QUEUE_LEN;
    } //TBD

    void Init();
    void Update();

    bool SendControllerMsg(JS_InteractiveData _controllerdata);

    JS_InteractiveData InteractiveData;
    JS_ClientCommand ClientCommand;
    serial::Serial *m_p_serialPort; 

};


class CustomControllerPacketHandler
{

private:

    InputMemoryStream m_InputStream;
    OutputMemoryStream m_OutputStream;
    uint8_t m_Buffer[30];
    CustomControllerPacketHandler();

public:

    float m_XPos;
    float m_YPos;
    float m_ZPos;
    float m_Roll;
    float m_Pitch;
    float m_Yaw;
    uint8_t m_HasTransform;

    static CustomControllerPacketHandler* Instance()
    {
        static CustomControllerPacketHandler instance;
        return &instance;
    }

    template<typename Stream> void SerializePacket(Stream &stream)
    {
        stream.Reset();
        stream.SerializeFloat(m_XPos);
        stream.SerializeFloat(m_YPos);
        stream.SerializeFloat(m_ZPos);
        stream.SerializeFloat(m_Roll);
        stream.SerializeFloat(m_Pitch);
        stream.SerializeFloat(m_Yaw);
        stream.SerializeU8(m_HasTransform);
    }

    void MoveToReceiveBuffer(const uint8_t* pData, const uint8_t _len);

    void OnPacketReceived();

    void SendPacket();
};


#endif
