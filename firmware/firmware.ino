#include <Arduino.h>
#include <algorithm>

extern "C" 
{
  #include "user_interface.h"
  #include "wifi_raw.h"
  #include "spi_slave.h"
}

#include "structures.h"

static int s_stats_last_tp = 0;

static constexpr uint8_t s_rate_mapping[15] = 
{ 
  0x0, //0 - B 1M   CCK
  0x1, //1 - B 2M   CCK
  0x5, //2 - G 2M   CCK Short Preamble
  0x2, //3 - B 5.5M CCK
  0x6, //4 - G 5.5M CCK Short Preamble
  0x3, //5 - B 11M  CCK
  0x7, //6 - G 11M  CCK Short Preamble
  0xB, //7 - G 6M   ODFM
  0xF, //8 - G 9M   ODFM
  0xA, //9 - G 12M  ODFM
  0xE, //A - G 18M  ODFM
  0x9, //B - G 24M  ODFM
  0xD, //C - G 36M  ODFM
  0x8, //D - G 48M  ODFM
  0xC, //E - G 54M  ODFM
};

/////////////////////////////////////////////////////////////////////////

static constexpr uint8_t s_filtered_mac[] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, };

/////////////////////////////////////////////////////////////////////////

static int s_uart_verbose = 0;
static char s_uart_command = 0;
static int s_uart_error_count = 0;

#define LOG(...) if (s_uart_verbose > 0) Serial.printf(__VA_ARGS__)

/////////////////////////////////////////////////////////////////////////

S2W_Packet s_wlan_packet;
float s_wlan_power_dBm = 0;

void set_wlan_power_dBm(float dBm)
{
  dBm = std::max(std::min(dBm, 20.5f), 0.f);
  s_wlan_power_dBm = dBm;
  system_phy_set_max_tpw(static_cast<uint8_t>(dBm * 4.f));
}

float get_wlan_power_dBm()
{
  return s_wlan_power_dBm;
}

void packet_sent_cb(uint8 status)
{
  {
    lock_guard lg;
  
    if (s_wlan_packet.ptr)
    {
      if (status == 0)
      {
        //int dt = micros() - s_send_start_time;
        //s_send_max_time = std::max(s_send_max_time, dt);
        //s_send_min_time = std::min(s_send_min_time, dt);
        s_stats.wlan_data_sent += s_wlan_packet.size + HEADER_SIZE;
      }
      else
      {
        LOG("WLAN send error");
        s_stats.wlan_error_count++;
      }
      end_reading_s2w_packet(s_wlan_packet);
      //xxx s_wlan_free_queue.push_and_clear(s_wlan_packet);
    }
    else
    {
      LOG("WLAN send missing packet");
      s_stats.wlan_error_count++;
    }
  }
  digitalWrite(LED_BUILTIN, HIGH);  
}

void packet_received_cb(struct RxPacket *pkt)
{
  uint16_t len = pkt->rx_ctl.legacy_length;
  if (len <= HEADER_SIZE)
  {
    LOG("WLAN receive header error");
    s_stats.wlan_error_count++;
    return;
  }
  
  //Serial.printf("Recv callback #%d: %d bytes\n", counter++, len);
  //Serial.printf("Channel: %d PHY: %d\n", pkt->rx_ctl.channel, wifi_get_phy_mode());

  //uint8_t broadcast_mac[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  //Serial.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  uint8_t* data = pkt->data;
  if (memcmp(data + 10, s_filtered_mac, 6) != 0)
  {
    return;
  }

  data += HEADER_SIZE;
  len -= HEADER_SIZE; //skip the 802.11 header

  len -= 4;//the received length has 4 more bytes at the end for some reason.

  int16_t rssi = pkt->rx_ctl.rssi;
  if (s_uart_verbose > 1)
  {
    Serial.printf("RSSI: %d, CH: %d, SZ: %d\n", rssi, pkt->rx_ctl.channel, len);
    if (s_uart_verbose > 2)
    {
      Serial.printf("---->\n");
      Serial.write(data, len);
      Serial.printf("\n<----\n");
    }
  }

  size_t size = std::min<size_t>(len, MAX_PAYLOAD_SIZE);

  uint8_t* buffer = nullptr;
  {
    lock_guard lg;
    buffer = s_w2s_queue.start_writing(size);
    //xxx p = s_spi_free_queue.pop();
  }
  
  if (buffer)
  {
    memcpy(buffer, data, size);
    //p->rssi = rssi;
    
    {
      lock_guard lg;
      s_w2s_queue.end_writing();
      //xxx s_spi_to_send_queue.push(p);
    }
  }
  else
  {
    s_stats.wlan_received_packets_dropped++;
  }
  s_stats.wlan_data_received += len;
}

/////////////////////////////////////////////////////////////////////////

S2W_Packet s_uart_packet;

void ICACHE_FLASH_ATTR parse_command()
{
  if (s_uart_command == 0)
  {
    char ch = Serial.read();
    if (ch <= 0 || ch == '\n')
    {
      return;
    }
    s_uart_command = ch;
  }

  lock_guard lg;

  int available = Serial.available();
  if (available <= 0)
  {
    return;
  }

  if (s_uart_command == 'V')
  {
    s_uart_command = 0;
    s_uart_verbose++;
    if (s_uart_verbose > 2)
    {
      s_uart_verbose = 0;
    }
    Serial.printf("Verbose: %d\n", s_uart_verbose);
  }
  else if (s_uart_command == 'S')
  {
    if (!s_uart_packet.ptr)
    {
      //xxx s_uart_packet = s_wlan_free_queue.pop();
    }
    if (!s_uart_packet.ptr)
    {
      LOG("Sending failed: previous packet still in flight\n");
      s_uart_command = 0;
      s_uart_error_count++;
      return;
    }
    while (available-- > 0)
    {
      char ch = Serial.read();
      if (ch == '\n')
      {
        s_uart_packet.size = s_uart_packet.offset;
        LOG("Sending packet of size %d\n", s_uart_packet.size);
        //xxx s_wlan_to_send_queue.push(s_uart_packet);
        s_uart_command = 0;
      }
      else
      {
        if (s_uart_packet.offset >= MAX_PAYLOAD_SIZE)
        {
          while (available-- > 0) Serial.read();
          LOG("Packet too big: %d > %d\n", s_uart_packet.offset + 1, MAX_PAYLOAD_SIZE);
          //xxx s_wlan_free_queue.push_and_clear(s_uart_packet);
          
          s_uart_command = 0;
          s_uart_error_count++;
          return;
        }
        //s_uart_packet->payload_ptr[s_uart_packet->offset++] = ch;
      }
    }
  }
  else if (s_uart_command == 'T')
  {
    s_uart_command = 0;
    //Serial.printf("Received: %d bytes, errors: %d\n", s_received, s_receive_error_count);
    //Serial.printf("Sent: %d bytes, errors: %d\n", s_sent, s_send_error_count);
    //Serial.printf("Command errors: %d\n", s_uart_error_count);
  }
  else if (s_uart_command == 'C')
  {
    if (available == 0)
    {
      return;
    }
    s_uart_command = 0;
    
    uint8_t channel = 0;
    char ch = Serial.read();
    if (ch >= '1' && ch <= '9') channel = ch - '0';
    else if (ch >= 'A' && ch <= 'F') channel = ch - 'A' + 10;
    else if (ch >= 'a' && ch <= 'f') channel = ch - 'a' + 10;
    else 
    {
      s_uart_error_count++;
      LOG("Command error: Illegal channel %c\n", ch);
      return;
    }
    if (wifi_set_channel(channel))
    {
      LOG("Channel set to %d\n", channel);
    }
    else
    {
      LOG("Command error: call to wifi_set_channel failed\n");
    }
  }
  else if (s_uart_command == 'P')
  {
    if (available == 0)
    {
      return;
    }
    s_uart_command = 0;
    
    float power = 0.f;
    char ch = Serial.read();
    if (ch >= '0' && ch <= '9') power = ((ch - '0') / 9.f) * 20.5f;
    else 
    {
      s_uart_error_count++;
      LOG("Command error: Illegal power %c\n", ch);
      return;
    }
    
    set_wlan_power_dBm(power);
    LOG("Power set to %.2f\n", power);
  }
  else if (s_uart_command == 'R')
  {
    if (available == 0)
    {
      return;
    }
    s_uart_command = 0;
  
    uint8_t rate = 0;
    char ch = Serial.read();
    if (ch >= '0' && ch <= '9') rate = ch - '0';
    else if (ch >= 'A' && ch <= 'E') rate = ch - 'A' + 10;
    else if (ch >= 'a' && ch <= 'e') rate = ch - 'a' + 10;
    else 
    {
      s_uart_error_count++;
      LOG("Command error: Illegal rate %c\n", ch);
      return;
    }
    if (wifi_set_user_fixed_rate(FIXED_RATE_MASK_ALL, s_rate_mapping[rate]) == 0)
    {
      LOG("Rate set to %d\n", rate);
    }
    else
    {
      LOG("Command error: call to wifi_set_user_fixed_rate failed\n");
    }
  }
  else
  {
    s_uart_error_count++;
    LOG("Command error: %c\n", s_uart_command);
    s_uart_command = 0;
  }
}

/////////////////////////////////////////////////////////////////////////

S2W_Packet s_spi_incoming_packet;
W2S_Packet s_spi_outgoing_packet;

enum SPI_Command : uint16_t
{
    SPI_CMD_SEND_PACKET = 1,
    SPI_CMD_GET_PACKET = 2,
    SPI_CMD_SET_RATE = 3,
    SPI_CMD_GET_RATE = 4,
    SPI_CMD_SET_CHANNEL = 5,
    SPI_CMD_GET_CHANNEL = 6,
    SPI_CMD_SET_POWER = 7,
    SPI_CMD_GET_POWER = 8,
    SPI_CMD_GET_STATS = 9,
};

uint32_t s_spi_temp_buffer[16];

void spi_on_data_received()
{
  lock_guard lg;
  
  if (s_spi_incoming_packet.ptr)
  {
    uint32_t poffset = s_spi_incoming_packet.offset;
    uint32_t psize = s_spi_incoming_packet.size;

    uint32_t size = psize - poffset;
    if (size > CHUNK_SIZE)
    {
      size = CHUNK_SIZE;
    }
    
    if (size == CHUNK_SIZE && ((size_t)(s_spi_incoming_packet.payload_ptr + poffset) & 3) == 0)
    {
      spi_slave_get_data((uint32_t*)(s_spi_incoming_packet.payload_ptr + poffset));
    }
    else
    {
      spi_slave_get_data(s_spi_temp_buffer);
      memcpy(s_spi_incoming_packet.payload_ptr + poffset, s_spi_temp_buffer, size);
    }
    
    s_spi_incoming_packet.offset += size;
    if (s_spi_incoming_packet.offset >= psize)
    {
      end_writing_s2w_packet(s_spi_incoming_packet);
      //s_stats.spi_packets_received++;
    }
    s_stats.spi_data_received += size;
  }
  else
  {
    s_stats.spi_error_count++;
    //Serial.printf("Unexpected data\n");
  }
}

void spi_on_data_sent()
{
  lock_guard lg;
  
  if (s_spi_outgoing_packet.ptr)
  {
    uint32_t psize = s_spi_outgoing_packet.size;
    uint32_t size = psize - s_spi_outgoing_packet.offset;
    if (size > CHUNK_SIZE)
    {
      size = CHUNK_SIZE;
    }
    s_spi_outgoing_packet.offset += size;
    if (s_spi_outgoing_packet.offset >= psize)
    {
      end_reading_w2s_packet(s_spi_outgoing_packet);
      //xxx s_spi_free_queue.push_and_clear(s_spi_outgoing_packet);
      //s_spi_packets_sent++;
    }
    else
    {
      size_t poffset = s_spi_outgoing_packet.offset;
      //prepare next transfer
      if (size == CHUNK_SIZE && ((size_t)(s_spi_outgoing_packet.ptr + poffset) & 3) == 0)
      {
        spi_slave_set_data((uint32_t*)(s_spi_outgoing_packet.ptr + poffset));
      }
      else
      {
        memcpy(s_spi_temp_buffer, s_spi_outgoing_packet.ptr + poffset, size);
        spi_slave_set_data(s_spi_temp_buffer);
      }
    }

    s_stats.spi_data_sent += size;
  }
  else
  {
    LOG("SPI send missing packet\n");
    s_stats.spi_error_count++;
  }
}

void spi_on_status_received(uint32_t status)
{
  lock_guard lg;

//  Serial.printf("spi status received sent: %d\n", status);
  SPI_Command command = (SPI_Command)(status >> 24);
  if (command == SPI_Command::SPI_CMD_SEND_PACKET)
  {
    uint32_t size = status & 0xFFFF;
    if (size > MAX_PAYLOAD_SIZE)
    {
      s_stats.spi_error_count++;
      LOG("Packet too big: %d\n", size);
    }
    else
    {
      if (!s_spi_incoming_packet.ptr)
      {
        start_writing_s2w_packet(s_spi_incoming_packet, size);
        //xxx s_spi_incoming_packet = s_wlan_free_queue.pop();
        if (!s_spi_incoming_packet.ptr)
        {
          s_stats.spi_error_count++;
          s_stats.spi_received_packets_dropped++;
          //Serial.printf("Not ready to send\n");
        }
      }
    }
    return;
  }
  else
  {
    if (s_spi_outgoing_packet.ptr)
    {
      LOG("SPI outgoing packet interrupted by %d, offset %d, size %d\n", command, s_spi_outgoing_packet.offset, s_spi_outgoing_packet.size);
      cancel_reading_w2s_packet(s_spi_outgoing_packet);
      //xxx s_spi_free_queue.push_and_clear(s_spi_outgoing_packet); //cancel the outgoing one
    }
  }
  
  if (command == SPI_Command::SPI_CMD_GET_PACKET)
  {
    if (!s_spi_outgoing_packet.ptr)
    {
      start_reading_w2s_packet(s_spi_outgoing_packet);
      //xxx s_spi_outgoing_packet = s_spi_to_send_queue.pop();
    }
    
    if (s_spi_outgoing_packet.ptr)
    {
      uint32_t size = s_spi_outgoing_packet.size & 0xFFFF;
      uint32_t rssi = 0;//*reinterpret_cast<uint8_t*>(&s_spi_outgoing_packet->rssi) & 0xFF;
      uint32_t status = (uint32_t(SPI_Command::SPI_CMD_GET_PACKET) << 24) | (rssi << 16) | size;
      spi_slave_set_status(status);

      if (size > CHUNK_SIZE)
      {
        size = CHUNK_SIZE;
      }
      if (size == CHUNK_SIZE && ((size_t)(s_spi_outgoing_packet.ptr) & 3) == 0)
      {
        spi_slave_set_data((uint32_t*)(s_spi_outgoing_packet.ptr));
      }
      else
      {
        memcpy(s_spi_temp_buffer, s_spi_outgoing_packet.ptr, size);
        spi_slave_set_data(s_spi_temp_buffer);
      }
    }
    else
    {
      spi_slave_set_status(0);
    }
    return;
  }
  else
  {
    if (s_spi_incoming_packet.ptr)
    {
      LOG("SPI incoming packet interrupted by %d\n", command);
      cancel_writing_s2w_packet(s_spi_incoming_packet);
      //xxx s_wlan_free_queue.push_and_clear(s_spi_incoming_packet); //cancel the incoming one
    }
  }
  
  if (command == SPI_Command::SPI_CMD_SET_RATE)
  {
    uint32_t rate = status & 0xFFFF;
    LOG("Setting rate: %d\n", rate);
    if (rate >= sizeof(s_rate_mapping) || wifi_set_user_fixed_rate(FIXED_RATE_MASK_ALL, s_rate_mapping[rate]) != 0)
    {
      LOG("Failed to set rate %d", rate);
      s_stats.spi_error_count++;
    }
  }
  else if (command == SPI_Command::SPI_CMD_GET_RATE)
  {
    uint8_t enable_mask = 0;
    uint8_t rate = 0;
    if (wifi_get_user_fixed_rate(&enable_mask, &rate) == 0)
    {
      uint16_t mapped_rate = 0xFFFF;
      for (uint16_t i = 0; i < sizeof(s_rate_mapping); i++)
      {
        if (rate == s_rate_mapping[i])
        {
          mapped_rate = i;
          break;
        }
      }
      if (mapped_rate == 0xFFFF)
      {
        LOG("Cannot map hardware rate %d", rate);
      }
      spi_slave_set_status((SPI_Command::SPI_CMD_GET_RATE << 24) | mapped_rate);
    }
    else
    {
      LOG("Cannot get rate");
      s_stats.spi_error_count++;
    }
  }
  else if (command == SPI_Command::SPI_CMD_SET_CHANNEL)
  {
    uint32_t channel = status & 0xFFFF;
    LOG("Setting channel: %d\n", channel);
    if (channel == 0 || channel > 11 || !wifi_set_channel(channel))
    {
      LOG("Cannot set channel %d", channel);
      s_stats.spi_error_count++;
    }
  }
  else if (command == SPI_Command::SPI_CMD_GET_CHANNEL)
  {
    uint8_t channel = wifi_get_channel();
    spi_slave_set_status((SPI_Command::SPI_CMD_GET_CHANNEL << 24) | channel);
  }
  else if (command == SPI_Command::SPI_CMD_SET_POWER)
  {
    uint16_t power = status & 0xFFFF;
    float dBm = (static_cast<float>(power) - 32767.f) / 100.f;
    LOG("Setting power: %f\n", dBm);
    set_wlan_power_dBm (dBm);
  }
  else if (command == SPI_Command::SPI_CMD_GET_POWER)
  {
    float dBm = get_wlan_power_dBm();
    uint16_t power = static_cast<uint16_t>(std::max(std::min((dBm * 100.f), 32767.f), -32767.f) + 32767.f);
    spi_slave_set_status((SPI_Command::SPI_CMD_GET_POWER << 24) | power);
  }
  else if (command == SPI_Command::SPI_CMD_GET_STATS)
  {
    uint32_t data[8] = { 0 };
    memcpy(data, &s_stats, sizeof(Stats));
    spi_slave_set_data(data);
  }
  else
  {
      LOG("Unknown command: %d\n", command);
  }
  //s_stats.spi_status_received++;
}

void spi_on_status_sent(uint32_t status)
{
  lock_guard lg;
  //LOG("Status sent: %d\n", status);
  spi_slave_set_status(0);
//  s_spi_status_sent++;
}

/////////////////////////////////////////////////////////////////////////

void setup() 
{
  Serial.begin(115200);
  Serial.setTimeout(999999);

  srand(millis());

/*  auto& queue = s_spi_to_wlan_queue;

  while (1)
  {
    uint8_t counter = 0;
    while (1)
    {
      size_t size = rand() % 1500 + 10;
//      Serial.printf("%d: Start writing %d...", (int)counter, size);
      uint8_t* buffer = queue.start_writing(size);
      if (!buffer)
      {
//        Serial.printf("failed\n");
        break;
      }
//      Serial.printf("ok\n");
      memset(buffer, counter, size);
      queue.end_writing();
      counter++;
    }

    counter = 0;
    while (1)
    {
      size_t size = 0;;
//      Serial.printf("%d: Start reading...", (int)counter);
      uint8_t* buffer = queue.start_reading(size);
      if (!buffer)
      {
//        Serial.printf("failed\n");
        break;
      }
//      Serial.printf("ok, size %d\n", size);
      for (size_t i = 0; i < size; i++)
      {
        if (buffer[i] != counter)
        {
          Serial.printf("corruption @ %d, expected %d, got %d\n", i, (int)counter, (int)buffer[i]);
          break;
        }
      }
      queue.end_reading();
      counter++;
    }
    delay(1);
  }
*/
  

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  bool ok = false;
  int res = 0;

  set_wlan_power_dBm(20.5f);
  
  //ok = wifi_set_opmode(STATION_MODE);
  if (!wifi_set_opmode(SOFTAP_MODE))
  {
    Serial.printf("Call to wifi_set_opmode failed\n");
  }

  if (!wifi_set_channel(1))
  {
    Serial.printf("Call to wifi_set_channel failed\n");
  }
  
  if (!wifi_set_phy_mode(PHY_MODE_11B))
  {
    Serial.printf("Call to wifi_set_phy_mode failed\n");
  }
  
  if (wifi_set_user_fixed_rate(FIXED_RATE_MASK_ALL, 2) != 0)
  {
    Serial.printf("Call to wifi_set_user_fixed_rate failed\n");
  }

  wifi_register_send_pkt_freedom_cb(&packet_sent_cb);
  wifi_set_raw_recv_cb(packet_received_cb);

  spi_slave_on_data(&spi_on_data_received);
  spi_slave_on_data_sent(&spi_on_data_sent);
  spi_slave_on_status(&spi_on_status_received);
  spi_slave_on_status_sent(&spi_on_status_sent);

  spi_slave_begin(4, nullptr);

  spi_slave_set_status(0);

  Serial.printf("Initialized\n");
}

void loop() 
{
  parse_command();

  if (!s_wlan_packet.ptr)
  {
    S2W_Packet p;
    {
      lock_guard lg;
      if (!s_wlan_packet.ptr)
      {
        start_reading_s2w_packet(s_wlan_packet);
        p = s_wlan_packet;
        //xxx s_wlan_packet = s_wlan_to_send_queue.pop();
      }
    }
    
    if (p.ptr)
    {
      digitalWrite(LED_BUILTIN, LOW);
      memcpy(p.ptr, s_packet_header, HEADER_SIZE);
      wifi_send_pkt_freedom(p.ptr, HEADER_SIZE + p.size, 0);   
//      lock_guard lg;
//      s_wlan_free_queue.push_and_clear(s_wlan_packet);

    }
  }

  if (s_uart_verbose > 0 && millis() - s_stats_last_tp >= 1000)
  {
    s_stats_last_tp = millis();
//    Serial.printf("Sent: %d bytes ec:%d, Received: %d bytes, SPI SS: %d, SPI SR: %d, SPI DS: %d, SPI DR: %d, SPI ERR: %d, SPI PD: %d\n", s_sent, s_send_error_count, s_received, s_spi_status_sent, s_spi_status_received, s_spi_data_sent, s_spi_data_received, s_spi_error_count, s_spi_packets_dropped);
    Serial.printf("WLAN S: %d, R: %d, E: %d, D: %d, %%: %d  SPI S: %d, R: %d, E: %d, D: %d, %%: %d\n", 
      s_stats.wlan_data_sent, s_stats.wlan_data_received, s_stats.wlan_error_count, s_stats.wlan_received_packets_dropped, s_s2w_queue.size() * 100 / s_s2w_queue.capacity(),
      s_stats.spi_data_sent, s_stats.spi_data_received, s_stats.spi_error_count, s_stats.spi_received_packets_dropped, s_w2s_queue.size() * 100 / s_w2s_queue.capacity());

    s_stats = Stats();

    //Serial.printf("Sent: %d bytes, min %dms, max %dms, ec: %d\n", s_sent, s_send_min_time, s_send_max_time, s_send_error_count);
    //s_sent = 0;
    //s_send_max_time = -999999;
    //s_send_min_time = 999999;
    //s_send_error_count = 0;
  }
  //*/

  //Serial.printf("Call to wifi_set_channel failed\n");
}
