#pragma once

#include <Arduino.h>
#include <MeshCore.h>

// Number of raw packets kept in RAM for `log hex`. Each entry is ~272 bytes.
// Set to 0 to compile the ring out.
#ifndef PACKET_LOG_RING_SIZE
  #if defined(STM32_PLATFORM)
    #define PACKET_LOG_RING_SIZE  8
  #else
    #define PACKET_LOG_RING_SIZE  64
  #endif
#endif

/**
 * \brief  RAM ring buffer of raw packets, for lossless capture over the serial CLI.
 *
 * Each entry is tagged with a per-boot random id and a sequence number, so a reader can dump
 * repeatedly, keep an overlap with `clearOlderThan()`, de-duplicate, and detect gaps.
 * When full, the oldest entry is overwritten and the `dropped` counter is incremented.
 */
class PacketLogRing {
public:
  enum Kind : uint8_t { RX = 0, TX = 1, TX_FAIL = 2 };

  void begin(uint16_t boot_id) { _boot_id = boot_id; }

  void add(Kind kind, const uint8_t* raw, int len, uint32_t timestamp,
           float snr = 0, float rssi = 0, float score = 0);

  /**
   * \brief  print a header line, then one line per entry (oldest first):
   *     # boot=3fa2 now=1791031386 count=2 dropped=0
   *     1791031386 3fa2-412 RX -7.25 -106 21 1500AB...
   *     1791031387 3fa2-413 TX 1500AB...
   */
  void dump(Print& out, uint32_t now) const;

  /**
   * \brief  remove entries more than 'secs' seconds older than 'now'.
   * \returns  number of entries removed; 'kept' receives the number remaining.
   */
  int clearOlderThan(uint32_t now, uint32_t secs, int* kept);

  int count() const { return _count; }

private:
#if PACKET_LOG_RING_SIZE > 0
  struct Entry {
    uint32_t timestamp;
    uint32_t seq;
    int16_t snr_x4;
    int16_t rssi;
    int16_t score;
    uint8_t kind;
    uint8_t len;
    uint8_t raw[MAX_TRANS_UNIT];
  };
  Entry _entries[PACKET_LOG_RING_SIZE];
#endif
  int _head = 0;     // index of the oldest entry
  int _count = 0;
  uint32_t _next_seq = 0;
  uint32_t _dropped = 0;
  uint16_t _boot_id = 0;
};
