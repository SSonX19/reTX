#include <Arduino.h>

//#include <iostream>

#include "crsf_util.h"
#include "crc.h"

namespace crsf {

static const GENERIC_CRC8 s_crc{CRSF_CRC_POLY};

crsf_packet_t pack_rc_channels_data(const RcChannelsData &data)
{
    crsf_packet_t pkt{};
    //rcPacket_t rc{};
    pkt.header.device_addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    pkt.header.frame_size = RCframeLength + 2;
    pkt.header.type = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;

    pkt.rc_channels.data.ch0 = data[0];
    pkt.rc_channels.data.ch1 = data[1];
    pkt.rc_channels.data.ch2 = data[2];
    pkt.rc_channels.data.ch3 = data[3];
    pkt.rc_channels.data.ch4 = data[4];
    pkt.rc_channels.data.ch5 = data[5];
    pkt.rc_channels.data.ch6 = data[6];
    pkt.rc_channels.data.ch7 = data[7];
    pkt.rc_channels.data.ch8 = data[8];
    pkt.rc_channels.data.ch9 = data[9];
    pkt.rc_channels.data.ch10 = data[10];
    pkt.rc_channels.data.ch11 = data[11];
    pkt.rc_channels.data.ch12 = data[12];
    pkt.rc_channels.data.ch13 = data[13];
    pkt.rc_channels.data.ch14 = data[14];
    pkt.rc_channels.data.ch15 = data[15];

    uint8_t crc = crsf::crc_gen().calc(pkt.header.type);
    crc = crsf::crc_gen().calc((uint8_t *)&pkt.rc_channels, RCframeLength, crc);

    pkt.rc_channels.crc = crc;

    return pkt;
}

#if 0
void pack_rc_channel_data(std::vector<uint8_t> &dst, const RcChannelsData &data)
{
    rcPacket_t pkt;
    pkt.header.device_addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    pkt.header.frame_size = RCframeLength + 2;
    pkt.header.type = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;

    pkt.channels.ch0 = data[0];
    pkt.channels.ch1 = data[1];
    pkt.channels.ch2 = data[2];
    pkt.channels.ch3 = data[3];
    pkt.channels.ch4 = data[4];
    pkt.channels.ch5 = data[5];
    pkt.channels.ch6 = data[6];
    pkt.channels.ch7 = data[7];
    pkt.channels.ch8 = data[8];
    pkt.channels.ch9 = data[9];
    pkt.channels.ch10 = data[10];
    pkt.channels.ch11 = data[11];
    pkt.channels.ch12 = data[12];
    pkt.channels.ch13 = data[13];
    pkt.channels.ch14 = data[14];
    pkt.channels.ch15 = data[15];

    uint8_t crc = s_crc.calc(pkt.header.type);
    crc = s_crc.calc((uint8_t *)&pkt.channels, RCframeLength, crc);

    if (dst.capacity() < (sizeof(pkt) + 1))
        dst.reserve(sizeof(pkt) + 1);
    dst.resize(sizeof(pkt));
    memcpy(dst.data(), &pkt, sizeof(pkt));
    dst.push_back(crc);
}

std::vector<uint8_t> pack_rc_channel_data(const RcChannelsData &data)
{
    std::vector<uint8_t> dst;
    dst.reserve(RCframeLength + 4); // rc data + type + crc + frame_size + device_address
    pack_rc_channel_data(dst, data);
    return dst;
}
#endif

const GENERIC_CRC8 &crc_gen()
{
    return s_crc;
}

void dump_packet(packet_view pkt)
{
#if 0
    std::cout << std::format("address: 0x{:X}/{}\n"
                             "len:     {}\n"
                             "  whole frame len: {}\n"
                             "  payload len:     {}\n"
                             "type:    {}/{}\n"
                             "CRC:     0x{:02X}/0x{:02X}\n",
                             pkt.device_address(), crsf::address_str(pkt.device_address()),
                             pkt.frame_size() - 2,
                             pkt.frame_size(),
                             pkt.payload_size(),
                             pkt.type(), crsf::type_str(pkt.type()),
                             pkt.crc(), pkt.crc_calc());
#else
  Serial.print("address: 0x"); Serial.print(pkt.device_address(), 16); Serial.print("/"); Serial.println(crsf::address_str(pkt.device_address()).data());
  Serial.print("len: "); Serial.println(pkt.frame_size() - 2);
  Serial.print("  whole frame len: "); Serial.println(pkt.frame_size());
  Serial.print("  payload len:     "); Serial.println(pkt.payload_size());
  Serial.print("type: "); Serial.print(pkt.type()); Serial.print("/"); Serial.println(crsf::type_str(pkt.type()).data());
  //Serial.print("CRC: 0x"); Serial.print(pkt.crc(), 16); Serial.print("/0x"); Serial.println(pkt.crc_calc(), 16);
#endif
}

} // ::crsf

