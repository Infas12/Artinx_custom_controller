#ifndef PACKET_HPP
#define PACKET_HPP

#include "HashTable.hpp"
#include <stdint.h>
#include "Observer.hpp"
#include "MemoryStream.hpp"
#include "Crc.hpp"

#define STREAM_CHANNEL_BUFFER_LEN 128
#define HEADER_LEN 5

class Packet : public Notifier
{
protected:
    uint8_t m_ProtocolId;
    uint8_t m_DataLen;
    InputMemoryStream m_InputStream;
    OutputMemoryStream m_OutputStream;
    uint8_t m_Buffer[256];
    uint16_t m_CallbackLen;
    uint32_t m_LastReceiveTick;

public:
    Packet(uint8_t _id);

    void MoveBuffer(const uint8_t *pData, const uint8_t _len);
    void UpdateTick(uint32_t _tick);

    virtual void OnPacketReceived() = 0;
    virtual void SendPacket() = 0;
    virtual void Registration() = 0;

    uint32_t GetLastTickReceived() { return m_LastReceiveTick; }

    template <typename Stream>
    void SerializeHeader(Stream &stream)
    {
        stream.Reset();

        uint8_t _magicNumver = 0xA5;
        stream.SerializeU8(_magicNumver);

        uint8_t _dataLen = m_DataLen;
        stream.SerializeU8(_dataLen);

        uint16_t _seq = 0x0000;
        stream.SerializeU16(_seq);

        uint8_t _crc = 0x00;
        if (!stream.IsInput())
        {
            _crc = Crc::Get_CRC8_Check_Sum(m_Buffer, 4, Crc::CRC8_INIT);
        }
        stream.SerializeU8(_crc);

        uint8_t _protocalId = m_ProtocolId;
        stream.SerializeU8(_protocalId);
    }

    template <typename Stream>
    void SerializeCrc16(Stream &stream)
    {
        if (stream.IsInput())
        {
            return;
        }

        uint16_t _crc16 = 0;
        _crc16 = Crc::Get_CRC16_Check_Sum(m_Buffer, m_DataLen + 6, Crc::CRC16_INIT);
        stream.SerializeU16(_crc16);
    }
};

class StreamChannel
{
private:
    HashTable<Packet *> m_PacketHandlerTable;
    uint8_t m_Buffer[STREAM_CHANNEL_BUFFER_LEN];
    uint8_t m_PacketBuffer[STREAM_CHANNEL_BUFFER_LEN];
    uint8_t m_HeaderBuffer[HEADER_LEN];
    uint8_t m_BufferFront;
    uint8_t m_BufferRear;
    uint8_t m_BufferLen;
    uint8_t m_PacketLen;
    uint8_t m_HeaderLen;
    uint8_t m_ExpectedLen;
    uint16_t m_Seq;
    bool m_CheckingHeader;

public:
    void HandlePacket();
    StreamChannel();
    void Enqueue(uint8_t *_pData, uint32_t _len);
    void PacketHandlerRegistration(uint32_t _protocolId, Packet *_handler);
    void Update();
};

class PacketManager
{
private:
    uint32_t m_ChannelNum;

protected:
    StreamChannel *m_pChannel;

public:
    PacketManager(uint32_t _channelNum);

    virtual void Update();
    void Enqueue(uint32_t _channel, uint8_t *_pData, uint32_t _len);
    void PacketHandlerRegistration(uint32_t _channel, uint32_t _protocolId, Packet *_handler);

    virtual void Init();
};

#endif
