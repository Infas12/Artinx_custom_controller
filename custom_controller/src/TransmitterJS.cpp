#include "TransmitterJS.hpp"
#include "Time.hpp"
#include <iostream>
const uint8_t TransmitterJS::Magic_Number = 0xA5;
const uint8_t TransmitterJS::Header_Length = 5;
const uint32_t TransmitterJS::Flush_Tick = 1; //for controller, 1 tick = 10ms. 

void TransmitterJS::Init()
{
}

void TransmitterJS::Update()
{
    if(Time::getTick() % 100 == 0)
    {
        m_headerperS = m_hs;
        m_hs = 0;
    }
    uint8_t _currentRear = m_RxQueueRear;

    while(m_RxQueueFront != _currentRear)
    {
        uint8_t _data = m_RxQuene[m_RxQueueFront];

        if(m_FrameBufferLen < JS_FRAME_BUFFER_LEN)
        {
            m_FrameBuffer[m_FrameBufferLen] = _data;
            ++m_FrameBufferLen;
            if(m_FrameBufferLen == m_ExpectedLen)
            {
                if(Crc::VerifyCrc16CheckSum(m_FrameBuffer, m_FrameBufferLen))
                {
                    HandlePacket();
                }
            }
        }
            
        if(m_CheckingHeader)
        {
            m_HeaderBuffer[m_HeaderBufferLen] = _data;
            ++m_HeaderBufferLen;

            if(m_HeaderBufferLen == Header_Length)
            {
                m_HeaderBufferLen = 0;
                m_CheckingHeader = false;

                if(Crc::VerifyCrc8CheckSum(m_HeaderBuffer, Header_Length))
                {
                    uint8_t* _ptr = m_HeaderBuffer + 1;
                    m_ExpectedLen = ArrayStreamHelper::ReadUint16(_ptr, false) + 9;
                    //if(Crc::VerifyCrc16CheckSum(m_FrameBuffer, m_FrameBufferLen - Header_Length))
                    //HandlePacket();

                    memcpy(m_FrameBuffer, m_HeaderBuffer, Header_Length);
                    m_FrameBufferLen = Header_Length;
                }
            }
        }

        if(_data == Magic_Number)
        {
            m_CheckingHeader = true;
            m_HeaderBufferLen = 0;
            m_HeaderBuffer[m_HeaderBufferLen] = _data;
            ++m_HeaderBufferLen;
        }

        m_RxQueueFront = (m_RxQueueFront + 1) % JS_RX_QUEUE_LEN;
    }

    if(Time::getTick() % Flush_Tick == 0)
    {
        FlushBuffer();
    }
}

void TransmitterJS::HandlePacket()
{
    memcpy(m_CurrentFrame, m_FrameBuffer, m_FrameBufferLen);

    uint8_t* _ptr = m_CurrentFrame + Header_Length;
    uint16_t _packetId = ArrayStreamHelper::ReadUint16(_ptr);

    switch (_packetId)
    {
    case JPT_InteractiveData:
        memcpy(&InteractiveData, _ptr, sizeof(InteractiveData));
        break;


    case JPT_ClientCommand:
        memcpy(&ClientCommand, _ptr, sizeof(ClientCommand));
        break;

    default:
        break;
    }

    ++m_hs;
}

void TransmitterJS::FlushBuffer()
{
    if(m_SendBufferLen == 0)
    {
        return;
    }

    if (!m_p_serialPort->isOpen() || m_p_serialPort == nullptr)
    {
        return;
    }

    m_p_serialPort->write(m_SendBuffer, m_SendBufferLen);
    m_SendBufferLen = 0;
}


bool TransmitterJS::SendControllerMsg(JS_InteractiveData _controllerdata)
{
    if(m_SendBufferLen + sizeof(_controllerdata) + 9 > JS_SEND_BUFFER_LEN)
    {
        return false;
    }

    uint8_t* _pStart = &m_SendBuffer[m_SendBufferLen];

    uint8_t _header[7];
    _header[0] = Magic_Number;
    uint16_t _realLen = sizeof(_controllerdata);
    memcpy(&_header[1], &_realLen, sizeof(uint16_t));
    _header[3] = 0;
    _header[4] = Crc::Get_CRC8_Check_Sum(_header, 4, Crc::CRC8_INIT);

    uint16_t _jpt_customId = JPT_InteractiveData;
    memcpy(&_header[5], &_jpt_customId, sizeof(uint16_t));

    memcpy(&m_SendBuffer[m_SendBufferLen], _header, sizeof(_header));
    m_SendBufferLen += sizeof(_header);

    memcpy(&m_SendBuffer[m_SendBufferLen], &_controllerdata, sizeof(_controllerdata));
    m_SendBufferLen += sizeof(_controllerdata);   

    uint16_t _crc16 = Crc::Get_CRC16_Check_Sum(
        _pStart,
        sizeof(_header) + sizeof(_controllerdata),
        Crc::CRC16_INIT
    );    

    memcpy(&m_SendBuffer[m_SendBufferLen], &_crc16, sizeof(_crc16));
    m_SendBufferLen +=sizeof(_crc16);
    return true;    

}

CustomControllerPacketHandler::CustomControllerPacketHandler() : 
    m_InputStream(m_Buffer),
    m_OutputStream(m_Buffer),
    m_Buffer{0},
    m_XPos(0.0), 
    m_YPos(0.0), 
    m_ZPos(0.0),
    m_Roll(0.0),
    m_Pitch(0.0),
    m_Yaw(0.0),
    m_HasTransform(0)
{
}

void CustomControllerPacketHandler::MoveToReceiveBuffer(const uint8_t* pData, const uint8_t _len)
{
    memcpy(m_Buffer, pData, _len);        
};

void CustomControllerPacketHandler::OnPacketReceived()
{
    SerializePacket(m_InputStream);
}

void CustomControllerPacketHandler::SendPacket()
{
    TransmitterJS::JS_InteractiveData _controllerdata;
    SerializePacket(m_OutputStream);
    memcpy(_controllerdata.data,m_Buffer,30);
    TransmitterJS::Instance()->SendControllerMsg(_controllerdata);
    std::cout << m_XPos << " " << m_YPos << " " << m_ZPos << " " 
              << m_Yaw  << " " << m_Pitch<< " " << m_Roll << std::endl;
}