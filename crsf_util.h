#pragma once

#include <cstdint>
#include <cstdlib>

#include <string_view>
#include <array>
#include <vector>

#include "crsf_protocol.h"
#include "crsf_packet.h"
#include "crc.h"

namespace crsf {

enum {
    CH0,
    CH1,
    CH2,
    CH3,
    CH4,
    CH5,
    CH6,
    CH7,
    CH8,
    CH9,
    CH10,
    CH11,
    CH12,
    CH13,
    CH14,
    CH15,
    CH_COUNT
};

using RcChannelsData = std::array<uint16_t, 16>;
//using RcChannelsData = uint16_t[16];

//void pack_rc_channel_data(std::vector<uint8_t> &dst, const RcChannelsData &data);
//std::vector<uint8_t> pack_rc_channel_data(const RcChannelsData &data);

crsf_packet_t pack_rc_channels_data(const RcChannelsData &data);

void dump_packet(packet_view pkt);

const GENERIC_CRC8& crc_gen();

static inline std::string_view address_str(uint8_t addr)
{
#define CASE(x) case x: return #x
    switch (addr) {
        CASE(CRSF_ADDRESS_BROADCAST);
        CASE(CRSF_ADDRESS_USB);
        CASE(CRSF_ADDRESS_TBS_CORE_PNP_PRO);
        CASE(CRSF_ADDRESS_RESERVED1);
        CASE(CRSF_ADDRESS_GPS);
        CASE(CRSF_ADDRESS_TBS_BLACKBOX);
        CASE(CRSF_ADDRESS_FLIGHT_CONTROLLER);
        CASE(CRSF_ADDRESS_RESERVED2);
        CASE(CRSF_ADDRESS_RACE_TAG);
        CASE(CRSF_ADDRESS_RADIO_TRANSMITTER);
        CASE(CRSF_ADDRESS_CRSF_RECEIVER);
        CASE(CRSF_ADDRESS_CRSF_TRANSMITTER);
        CASE(CRSF_ADDRESS_ELRS_LUA);
    default:
        return "Unknown";
    }
#undef CASE
}

static inline std::string_view type_str(uint8_t type)
{
#define CASE(x) case x: return #x
    switch (type) {
        CASE(CRSF_FRAMETYPE_GPS);
        CASE(CRSF_FRAMETYPE_VARIO);
        CASE(CRSF_FRAMETYPE_BATTERY_SENSOR);
        CASE(CRSF_FRAMETYPE_BARO_ALTITUDE);
        CASE(CRSF_FRAMETYPE_HEARTBEAT);
        CASE(CRSF_FRAMETYPE_LINK_STATISTICS);
        CASE(CRSF_FRAMETYPE_OPENTX_SYNC);
        CASE(CRSF_FRAMETYPE_RADIO_ID);
        CASE(CRSF_FRAMETYPE_RC_CHANNELS_PACKED);
        CASE(CRSF_FRAMETYPE_ATTITUDE);
        CASE(CRSF_FRAMETYPE_FLIGHT_MODE);
        // Extended
        CASE(CRSF_FRAMETYPE_DEVICE_PING);
        CASE(CRSF_FRAMETYPE_DEVICE_INFO);
        CASE(CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY);
        CASE(CRSF_FRAMETYPE_PARAMETER_READ);
        CASE(CRSF_FRAMETYPE_PARAMETER_WRITE);

        //CASE(CRSF_FRAMETYPE_ELRS_STATUS);

        CASE(CRSF_FRAMETYPE_COMMAND);
        // Kiss frames
        CASE(CRSF_FRAMETYPE_KISS_REQ);
        CASE(CRSF_FRAMETYPE_KISS_RESP);
        // MSP
        CASE(CRSF_FRAMETYPE_MSP_REQ);
        CASE(CRSF_FRAMETYPE_MSP_RESP);
        CASE(CRSF_FRAMETYPE_MSP_WRITE);
        // Ardupilot
        CASE(CRSF_FRAMETYPE_ARDUPILOT_RESP);
    default:
        return "Unknown";
    }
#undef CASE
}

} // ::crsf
