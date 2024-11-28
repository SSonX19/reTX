#pragma once

#include <memory>
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "crsf_protocol.h"

namespace crsf {

class packet_view
{
public:
    packet_view(const crsf_packet_t &pkt);

    const crsf_packet_t *raw() const;

    uint8_t crc() const;
    uint8_t crc_calc() const;
    bool crc_valid() const;
    size_t payload_size() const;
    size_t frame_size() const; // whole frame size
    uint8_t device_address() const;
    uint8_t type() const;

    template<typename T, size_t Length = sizeof(T)>
    T get_value(size_t offset) const
    {
        if (offset + Length > sizeof(_pkt.payload))
          assert(0 && "offset to big");
        T value;
        memcpy(std::addressof(value), _pkt.payload + offset, Length);
        return value;
    }

private:
    const crsf_packet_t &_pkt;
};


#if __cpp_exceptions
void crsf_throw_packet_mismatch(uint8_t expected, uint8_t actual);

class packet_type_mismatch_exception : public std::runtime_error
{
public:
    packet_type_mismatch_exception(uint8_t expected, uint8_t actual)
        : std::runtime_error(std::format("expected frame type {:02X} vs {:02X}", expected, actual))
    {}
};
#else
static inline void crsf_throw_packet_mismatch(uint8_t /*expected*/, uint8_t /*actual*/)
{
  assert(0 && "crsf_throw_packet_mismatch");
}
#endif // __cpp_exceptions


static constexpr uint8_t PayloadSizeDynamic = 0;

template<class T, uint8_t Type, uint8_t PayloadSize = PayloadSizeDynamic>
class packet
{
public:
#if 0
    using RawData = std::array<uint8_t, CRSF_MAX_PACKET_LEN>;
#else
    struct RawData_s
    {
      const void *data() const noexcept {
        return _data;
      }
      void *data() noexcept {
        return _data;
      }
      uint8_t _data[CRSF_MAX_PACKET_LEN]{};
    };

    using RawData = RawData_s;
#endif
    static constexpr uint8_t type = Type;

    packet()
    {
        _pkt.header.type = Type;
        _pkt.header.frame_size = PayloadSize + CRSF_FRAME_LENGTH_TYPE_CRC;
        init();
    }

    packet(crsf_packet_t pkt)
        : _pkt(std::move(pkt))
    {
        if (Type != _pkt.header.type)
            crsf_throw_packet_mismatch(Type, _pkt.header.type);
        init();
    }

    packet(const RawData &data)
    {
        if (Type != data[CRSF_TELEMETRY_TYPE_INDEX])
            crsf_throw_packet_mismatch(Type, data[CRSF_TELEMETRY_TYPE_INDEX]);
        memcpy(&_pkt, data.data(), sizeof(_pkt));
        init();
    }

#if 0
    packet(const std::span<uint8_t> &data)
    {
        if (data.size() < CRSF_TELEMETRY_TYPE_INDEX+1 && Type != data[CRSF_TELEMETRY_TYPE_INDEX])
            crsf_throw_packet_mismatch(Type, data[CRSF_TELEMETRY_TYPE_INDEX]);
        memcpy(&_pkt, data.data(), std::min(sizeof(_pkt), data.size()));
        init();
    }
#endif

    const crsf_packet_t *raw() const
    {
        return &_pkt;
    }

    // useful for use with asio::buffer
#if 0
    std::span<const uint8_t> span() const
    {
        return {reinterpret_cast<const uint8_t*>(raw()),
                frame_size()};
    }
#endif

    packet_view view() const
    {
        return _pkt;
    }

    size_t payload_size() const
    {
        return view().payload_size();
    }

    size_t frame_size() const // whole frame size
    {
        return view().frame_size();
    }

    bool crc_valid() const
    {
        return view().crc_valid();
    }

    void set_device_addr(uint8_t dest)
    {
        _pkt.header.device_addr = dest;
    }

    void finalize()
    {
        // if constexpr (PayloadSize == PayloadSizeDynamic) {
        //     // TBD: update size, ask up to hierarcy
        // }
        auto const crc_idx = payload_size();
        _pkt.payload[crc_idx] = packet_view(_pkt).crc_calc();
    }

protected:

    void init()
    {
        static_cast<T*>(this)->do_init();
    }

    void do_init()
    {
        // do nothing
    }

    template<typename T1, size_t Length = sizeof(T1)>
    void set_value(size_t offset, T1 value)
    {
        if (offset + Length > sizeof(_pkt.payload))
            //throw std::runtime_error("offset to big");
            assert(0 && "offset to big");
        memcpy(_pkt.payload + offset, std::addressof(value), Length);
    }

    template<typename T1, size_t Length = sizeof(T1)>
    T get_value(size_t offset) const
    {
        if (offset + Length > sizeof(_pkt.payload))
            // throw std::runtime_error("offset to big");
            assert(0 && "offset to big");
        T value;
        memcpy(std::addressof(value), _pkt.payload + offset, Length);
        return value;
    }

protected:
    crsf_packet_t _pkt{};
};



class packet_heartbeat : public packet<packet_heartbeat, CRSF_FRAMETYPE_HEARTBEAT, CRSF_FRAME_HEARTBEAT_PAYLOAD_SIZE>
{
public:
    using Base = packet<packet_heartbeat, CRSF_FRAMETYPE_HEARTBEAT, CRSF_FRAME_HEARTBEAT_PAYLOAD_SIZE>;

    // import base constructors
    using Base::Base;

    int16_t origin() const;
    void set_origin(int16_t new_origin);

};


class packet_video_start : public packet<packet_video_start, 0xFE, 2>
{
public:
    enum {
        OffsetPort = 0x00,
        OffsetCrc = OffsetPort + sizeof(uint16_t),
    };

    class view : public packet_view {
    public:
        using packet_view::packet_view;
        uint16_t port() const;
    };

    using Base = packet<packet_video_start, 0xFE, 2>;
    using Base::Base;

    view get_view() const;

    uint16_t port() const;
    void set_port(uint16_t port);
};


class packet_command_receiver_id : public packet<packet_command_receiver_id, CRSF_FRAMETYPE_COMMAND>
{
public:
    enum {
        OffsetExtDst = 0x00,
        OffsetExtSrc = OffsetExtDst + sizeof(uint8_t),
        OffsetCommand = OffsetExtSrc + sizeof(uint8_t),
        OffsetSubCommand = OffsetCommand + sizeof(uint8_t),
        OffsetCommandModelSeletId = OffsetSubCommand + sizeof(uint8_t),
    };

    class view : public packet_view {
        using packet_view::packet_view;
        uint8_t ext_dst() const;
        uint8_t ext_src() const;
        uint8_t command() const;
        uint8_t subcommand() const;
        uint8_t receiver_id() const;
    };

    using Base = packet<packet_command_receiver_id, CRSF_FRAMETYPE_COMMAND>;
    using Base::Base;

    view get_view() const;

    void set_ext_dst(uint8_t dst);
    void set_ext_src(uint8_t src);

    void set_receiver_id(uint8_t id);

    // TBD:
    void do_init();
};


} // namespace crsf
