#include <cassert>
#include <cstdio>

#include "crsf_protocol.h"
#include "crsf_packet.h"
#include "crsf_util.h"

static constexpr int BAUD_RATE = 400'000;
//static constexpr int BAUD_RATE_RX = BAUD_RATE;
static constexpr int BAUD_RATE_RX = 420'000;
static constexpr int BAUD_RATE_TX = BAUD_RATE;
static constexpr bool TX_HALF_DUPLEX = true;

// UART1 and UART3 both 5v tolerant
HardwareSerial Serial1{USART1}; // to avoid conflict with Serial, witch can be USB CDC
HardwareSerial Serial3{USART3};

template<size_t Size>
struct buffer
{
  size_t put(const void *src, size_t count)
  {
      if (Size - _count < count) {
        count = Size - _count;
      }

      if (!count)
        return count;

      memcpy(_buffer + _count, src, count);
      _count += count;
      
      return count;
  }

  size_t put(int ch)
  {
    if (ch < 0)
      return 0;
    if (!(Size - _count))
      return 0;

    _buffer[_count] = static_cast<uint8_t>(ch);
    _count++;

    return 1;
  }

  size_t count() const noexcept
  {
    return _count;
  }

  static constexpr
  size_t size() noexcept
  {
    return Size;
  }

  void clear()
  {
    _count = 0;
    memset(_buffer, 0, Size);
  }

  uint8_t operator[](size_t index) const
  {
    assert(index < _count);
    return _buffer[index];
  }

  uint8_t& operator[](size_t index)
  {
    assert(index < _count);
    return _buffer[index];
  }

  const uint8_t* data() const
  {
    return _buffer;
  }

  uint8_t* data()
  {
    return _buffer;
  }

  uint8_t _buffer[Size]{};
  size_t _count{};
};  

namespace crsf {

// any other implementation are welcomed
using crsf_buffer = buffer<CRSF_MAX_PACKET_LEN>;

enum class Offset : size_t
{
  DeviceAddress = 0,
  FrameLength = 1,
  Type = 2,
  PayloadStart = 3,

  // PayloadSpecific
};

size_t get_frame_length(const crsf_buffer &buf)
{
  return buf.count() >= 2 ? buf[(size_t)Offset::FrameLength] : 0;
}

size_t get_payload_length(const crsf_buffer &buf)
{
  auto const frame_len = get_frame_length(buf);
  return (frame_len >= 2) ? frame_len - 2 : 0;
}

size_t get_packet_length(const crsf_buffer &buf)
{
  return buf.count() >= 2 ? get_frame_length(buf) + 2 : 0;
}

size_t get_crc_offset(const crsf_buffer &buf)
{
  if (buf.count() < 2)
    return 0;

  return get_frame_length(buf) + 1;
}

bool is_complete(const crsf_buffer &buf)
{
  // <ADDR><FRAME LEN><TYPE><PAYLOAD><CRC>
  if (buf.count() < 4) // at least ADDR, LEN, TYPE and CRC
    return false;
  return buf.count() == get_packet_length(buf);
}

} // ::anonymous

struct app_context 
{
  app_context() 
  {
    rc_data_init();
  }

  void rc_data_init()
  {
    for (auto &ch : rc_channel_data) {
      ch = CRSF_CHANNEL_VALUE_MID;
    }
  }

  unsigned long _send_timeout = 0;

  crsf::crsf_buffer crsf_buffer_rx; // from Serial3 to Serial - RC Data and control
  crsf::crsf_buffer crsf_buffer_tx; // from Serial to Serial3 - Thelemetry


  //crsf::crsf_buffer crsf_pending_packet_rx;
  crsf_packet_t crsf_pending_packet_rx;
  bool pending_packet_rx = false;

  crsf::RcChannelsData rc_channel_data;
  uint32_t rc_channel_data_tm = 0;
};

app_context s_ctx;

void setup() 
{
  // Serial must be mapped into USBSerial
  Serial.begin(115200);

  // HAL_HalfDuplex_EnableReceiver(USART1);
  // uart_enable_rx
  // uart_enable_tx
  if (TX_HALF_DUPLEX)
    Serial1.setHalfDuplex(); // make RX NC
  Serial1.begin(BAUD_RATE_TX);
  
  Serial3.begin(BAUD_RATE_RX);


  Serial.println("Retransmitter proxy started");
}

void loop() 
{
  // Process incoming data from the Receiver (RX)
  while (Serial3.available()) {
    auto const data = Serial3.read();
    if (data < 0)
      break;

    //Serial.print("RX byte: 0x");
    //Serial.println(data, 16);

    if (s_ctx.crsf_buffer_rx.count() == s_ctx.crsf_buffer_rx.size()) {
      Serial.println("RX: drop wrong packet");

      crsf_packet_t pkt{};
      memcpy(&pkt, s_ctx.crsf_buffer_rx.data(), sizeof(pkt));
      crsf::dump_packet(pkt);

      s_ctx.crsf_buffer_rx.clear();
    }
    
    s_ctx.crsf_buffer_rx.put(data);

    if (crsf::is_complete(s_ctx.crsf_buffer_rx)) {

      //Serial.println("RX: packet done");

      auto &pkt = s_ctx.crsf_pending_packet_rx;
      auto view = crsf::packet_view(pkt);
      memcpy(&pkt, s_ctx.crsf_buffer_rx.data(), s_ctx.crsf_buffer_rx.count());
      // Start next packet reading
      s_ctx.crsf_buffer_rx.clear();

      // TBD: properly process it
      // - RC Data must be collected into buffer for scheduled and repeatelly sending
      // - Other packets scheduled to send into Transmitter
      // - Some (???) packets must  be filtered. Like Link Statistics?
      // - Check CRC and drop wrong packets

      // Drop packet with wrong CRC
      if (!view.crc_valid()) {
        Serial.println("RX: wrong CRC");
      } else if (view.type() == CRSF_FRAMETYPE_LINK_STATISTICS) {
        //Serial.println("RX: drop LINK STATISTICS packet");
      } else if (view.type() == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
        // Unpack RC channel data and keep it for next sending
        s_ctx.rc_channel_data[crsf::CH0] = pkt.rc_channels.data.ch0;
        s_ctx.rc_channel_data[crsf::CH1] = pkt.rc_channels.data.ch1;
        s_ctx.rc_channel_data[crsf::CH2] = pkt.rc_channels.data.ch2;
        s_ctx.rc_channel_data[crsf::CH3] = pkt.rc_channels.data.ch3;
        s_ctx.rc_channel_data[crsf::CH4] = pkt.rc_channels.data.ch4;
        s_ctx.rc_channel_data[crsf::CH5] = pkt.rc_channels.data.ch5;
        s_ctx.rc_channel_data[crsf::CH6] = pkt.rc_channels.data.ch6;
        s_ctx.rc_channel_data[crsf::CH7] = pkt.rc_channels.data.ch7;
        s_ctx.rc_channel_data[crsf::CH8] = pkt.rc_channels.data.ch8;
        s_ctx.rc_channel_data[crsf::CH9] = pkt.rc_channels.data.ch9;
        s_ctx.rc_channel_data[crsf::CH10] = pkt.rc_channels.data.ch10;
        s_ctx.rc_channel_data[crsf::CH11] = pkt.rc_channels.data.ch11;
        s_ctx.rc_channel_data[crsf::CH12] = pkt.rc_channels.data.ch12;
        s_ctx.rc_channel_data[crsf::CH13] = pkt.rc_channels.data.ch13;
        s_ctx.rc_channel_data[crsf::CH14] = pkt.rc_channels.data.ch14;
        s_ctx.rc_channel_data[crsf::CH15] = pkt.rc_channels.data.ch15;

        s_ctx.rc_channel_data_tm = millis();
      } else {
        s_ctx.pending_packet_rx = true;
        //s_ctx.crsf_pending_packet_rx = s_ctx.crsf_buffer_rx;
      }

      // Make a chance to process current packet
      break;
    }
  } 

  static uint32_t _tx_model_id_tm = 0; // Model ID send to Transmitter (TX)
  static uint32_t _tx_rc_data_tm = 0;  // RC Data send to Transmitter (TX)
  if (_tx_model_id_tm + 1000 < millis()) {
    _tx_model_id_tm = millis();

    crsf::packet_command_receiver_id command_receiver_id;
    command_receiver_id.set_ext_dst(CRSF_ADDRESS_CRSF_TRANSMITTER);  // maybe CRSF_ADDRESS_BROADCAST
    command_receiver_id.set_ext_src(CRSF_ADDRESS_RADIO_TRANSMITTER); // maybe CRSF_ADDRESS_ELRS_LUA
    command_receiver_id.set_receiver_id(0x14); // just like a sample https://github.com/ExpressLRS/ExpressLRS/wiki/CRSF-Protocol
    command_receiver_id.finalize();

    Serial1.write(reinterpret_cast<const uint8_t*>(command_receiver_id.raw()),
                  command_receiver_id.frame_size());

  } else if (_tx_rc_data_tm + 10 < millis()) {
    _tx_rc_data_tm = millis();

    auto const pkt = crsf::pack_rc_channels_data(s_ctx.rc_channel_data);
    auto const size = crsf::packet_view(pkt).frame_size();

    Serial1.write(reinterpret_cast<const uint8_t*>(&pkt), size);
  } else if (s_ctx.pending_packet_rx) {
    // Send any other packet to TX if any
    s_ctx.pending_packet_rx = false;

    // Serial1.availableForWrite();
    Serial1.write(reinterpret_cast<const uint8_t*>(&s_ctx.crsf_pending_packet_rx),
                  crsf::packet_view(s_ctx.crsf_pending_packet_rx).frame_size());
  }


  // Reset RC data if no update for long time
  if (s_ctx.rc_channel_data_tm + 500 < millis()) {
    //s_ctx.rc_data_init();
    s_ctx.rc_channel_data_tm = millis();
    Serial.println("RX: warn: too old RC data");
  }

}
