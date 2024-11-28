//#include <cstring>

#include "crsf_packet.h"
#include "crsf_util.h"

#if __cpp_exceptions

void crsf_throw_packet_mismatch(uint8_t expected, uint8_t actual)
{
    throw crsf::packet_type_mismatch_exception(expected, actual);
}

#endif // __cpp_exceptions


namespace crsf {

#if 0
packet::packet() {}

packet::packet(crsf_packet_t pkt)
    : _pkt(std::move(pkt))
{
}

packet::packet(const RawData &data)
{
    memcpy(&_pkt, data.data(), sizeof(_pkt));
}

packet::packet(const std::span<uint8_t> &data)
{
    memcpy(&_pkt, data.data(), std::min(sizeof(_pkt), data.size()));
}

packet_view packet::view() const
{
    return _pkt;
}

bool packet::crc_valid() const
{
    return packet_view(_pkt).crc_valid();
}

size_t packet::payload_size() const
{
    return packet_view(_pkt).payload_size();
}
#endif

packet_view::packet_view(const crsf_packet_t &pkt)
    : _pkt(pkt)
{
}

const crsf_packet_t *packet_view::raw() const
{
    return &_pkt;
}

bool packet_view::crc_valid() const
{
    if (frame_size() < (CRSF_FRAME_LENGTH_ADDRESS+CRSF_FRAME_LENGTH_FRAMELENGTH+CRSF_FRAME_LENGTH_TYPE) ||
        frame_size() >= CRSF_MAX_PACKET_LEN)
        return false;
    auto const pkt_crc = crc();

    auto crc = crc_gen().calc(_pkt.header.type);
    crc = crc_gen().calc(_pkt.payload, payload_size(), crc);

    return crc == pkt_crc;
}

size_t packet_view::payload_size() const
{
    return _pkt.header.frame_size ? _pkt.header.frame_size - CRSF_FRAME_LENGTH_TYPE_CRC : 0;
}

size_t packet_view::frame_size() const
{
    return _pkt.header.frame_size + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH;
}

uint8_t packet_view::device_address() const
{
    return _pkt.header.device_addr;
}

uint8_t packet_view::type() const
{
    return _pkt.header.type;
}

uint8_t packet_view::crc() const
{
    return (frame_size() > 2 && frame_size() < CRSF_MAX_PACKET_LEN) ?
               _pkt.payload[payload_size()] : 0x00;
}

uint8_t packet_view::crc_calc() const
{
    auto crc = crc_gen().calc(_pkt.header.type);
    return crc_gen().calc(_pkt.payload, payload_size(), crc);
}

int16_t packet_heartbeat::origin() const
{
    return _pkt.heartheat.orig;
}

void packet_heartbeat::set_origin(int16_t new_origin)
{
    _pkt.heartheat.orig = new_origin;
}

packet_video_start::view packet_video_start::get_view() const
{
    return {_pkt};
}

uint16_t packet_video_start::port() const
{
    return get_view().port();
}

void packet_video_start::set_port(uint16_t port)
{
    set_value(OffsetPort, port);
}

uint16_t packet_video_start::view::port() const
{
    return get_value<uint16_t>(OffsetPort);
}

packet_command_receiver_id::view packet_command_receiver_id::get_view() const
{
    return {_pkt};
}

void packet_command_receiver_id::set_ext_dst(uint8_t dst)
{
    set_value(OffsetExtDst, dst);
}

void packet_command_receiver_id::set_ext_src(uint8_t src)
{
    set_value(OffsetExtSrc, src);
}

void packet_command_receiver_id::set_receiver_id(uint8_t id)
{
    set_value(OffsetCommandModelSeletId, id);
}

void packet_command_receiver_id::do_init()
{
    _pkt.header.frame_size += 5;
    set_value<uint8_t>(OffsetCommand, SUBCOMMAND_CRSF);
    set_value<uint8_t>(OffsetSubCommand, COMMAND_MODEL_SELECT_ID);
    set_device_addr(CRSF_ADDRESS_FLIGHT_CONTROLLER); // might be same for any Ext Command packet
}

uint8_t packet_command_receiver_id::view::ext_dst() const
{
    return get_value<uint8_t>(OffsetExtDst);
}

uint8_t packet_command_receiver_id::view::ext_src() const
{
    return get_value<uint8_t>(OffsetExtSrc);
}

uint8_t packet_command_receiver_id::view::command() const
{
    return get_value<uint8_t>(OffsetCommand);
}

uint8_t packet_command_receiver_id::view::subcommand() const
{
    return get_value<uint8_t>(OffsetSubCommand);
}

uint8_t packet_command_receiver_id::view::receiver_id() const
{
    return get_value<uint8_t>(OffsetCommandModelSeletId);
}

} // namespace crsf
