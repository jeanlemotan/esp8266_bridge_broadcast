#pragma once

#include <cassert>

constexpr size_t CHUNK_SIZE = 64;

constexpr size_t HEADER_SIZE = 24;
constexpr size_t MAX_PAYLOAD_SIZE = 1400 - HEADER_SIZE;
constexpr size_t MAX_PAYLOAD_SIZE_CHUNK_PADDED = MAX_PAYLOAD_SIZE + 32;

struct WLAN_Packet
{
  uint32_t storage[(HEADER_SIZE + MAX_PAYLOAD_SIZE_CHUNK_PADDED) / 4];
  uint8_t* ptr = (uint8_t*)&storage;
  uint8_t* payload_ptr = ptr + HEADER_SIZE;
  uint16_t size = 0;
  uint16_t offset = 0;
};

static_assert(MAX_PAYLOAD_SIZE_CHUNK_PADDED % CHUNK_SIZE == 0, "Size is not padded to chunk boundary");

struct SPI_Packet
{
  uint32_t storage[MAX_PAYLOAD_SIZE / 4 + 1];
  uint8_t* ptr = (uint8_t*)&storage;
  uint8_t* payload_ptr = ptr;
  uint16_t size = 0;
  uint16_t offset = 0;
  int8_t rssi = 0;
};

/////////////////////////////////////////////////////////////////////////

constexpr uint8_t MAX_WLAN_PACKET_COUNT = 10;
WLAN_Packet s_wlan_packets[MAX_WLAN_PACKET_COUNT];

constexpr uint8_t MAX_SPI_PACKET_COUNT = 6;
SPI_Packet s_spi_packets[MAX_SPI_PACKET_COUNT];

template<typename T, size_t N>
struct Queue
{
  Queue()
  {
    memset(m_packets, 0, sizeof(T*) * N);
  }

  inline uint8_t size() const __attribute__((always_inline))
  {
    return m_size;
  }
  inline bool empty() const __attribute__((always_inline))
  {
    return m_size == 0;
  }
  inline bool full() const __attribute__((always_inline))
  {
    return m_size >= N;
  }
  
  inline T* pop() __attribute__((always_inline))
  {
    if (m_size > 0)
    {
      return m_packets[--m_size];
    }
    return nullptr;
  }
  inline void push(T*& packet) __attribute__((always_inline))
  {
//    assert(m_size < N);
    T* p = packet;
    packet = nullptr;
    m_packets[m_size++] = p;
  }

  inline void push_and_clear(T*& packet) __attribute__((always_inline))
  {
//    assert(m_size < N);
    T* p = packet;
    packet = nullptr;
    p->size = 0;
    p->offset = 0;
    m_packets[m_size++] = p;
  }

  T* m_packets[N];
  uint8_t m_size = 0;
};

Queue<WLAN_Packet, MAX_WLAN_PACKET_COUNT> s_wlan_free_queue;
Queue<WLAN_Packet, MAX_WLAN_PACKET_COUNT> s_wlan_to_send_queue;

Queue<SPI_Packet, MAX_SPI_PACKET_COUNT> s_spi_free_queue;
Queue<SPI_Packet, MAX_SPI_PACKET_COUNT> s_spi_to_send_queue;


struct lock_guard
{
  inline lock_guard() __attribute__((always_inline))
  {
    noInterrupts();
  }
  inline ~lock_guard() __attribute__((always_inline))
  {
    interrupts();
  }
  lock_guard(const lock_guard&) = delete;
  lock_guard& operator=(const lock_guard&) = delete;
};


struct Stats
{
    uint32_t wlan_data_sent = 0;
    uint32_t wlan_data_received = 0;
    uint16_t wlan_error_count = 0;
    uint16_t wlan_received_packets_dropped = 0;
    uint32_t spi_data_sent = 0;
    uint32_t spi_data_received = 0;
    uint16_t spi_error_count = 0;
    uint16_t spi_received_packets_dropped = 0;
};
static_assert(sizeof(Stats) == 6 * 4, "Stats too big");

Stats s_stats;
