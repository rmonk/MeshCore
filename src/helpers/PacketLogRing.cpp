#include "PacketLogRing.h"
#include <math.h>

static const char* KIND_NAMES[] = { "RX", "TX", "TXF" };

static void printHexBytes(Print& out, const uint8_t* src, int len) {
  static const char HEX_CHARS[] = "0123456789ABCDEF";
  for (int i = 0; i < len; i++) {
    out.print(HEX_CHARS[src[i] >> 4]);
    out.print(HEX_CHARS[src[i] & 0x0F]);
  }
}

void PacketLogRing::add(Kind kind, const uint8_t* raw, int len, uint32_t timestamp, float snr, float rssi, float score) {
#if PACKET_LOG_RING_SIZE > 0
  if (len <= 0) return;
  if (len > MAX_TRANS_UNIT) len = MAX_TRANS_UNIT;

  int idx;
  if (_count < PACKET_LOG_RING_SIZE) {
    idx = (_head + _count) % PACKET_LOG_RING_SIZE;
    _count++;
  } else {
    idx = _head;   // overwrite oldest
    _head = (_head + 1) % PACKET_LOG_RING_SIZE;
    _dropped++;
  }
  Entry& e = _entries[idx];
  e.timestamp = timestamp;
  e.seq = _next_seq++;
  e.kind = kind;
  e.snr_x4 = (int16_t) lroundf(snr * 4);
  e.rssi = (int16_t) lroundf(rssi);
  e.score = (int16_t) lroundf(score * 1000);
  e.len = (uint8_t) len;
  memcpy(e.raw, raw, len);
#endif
}

void PacketLogRing::dump(Print& out, uint32_t now) const {
  char tmp[64];
  sprintf(tmp, "# boot=%04x now=%lu count=%d dropped=%lu\n", _boot_id, (unsigned long)now, _count, (unsigned long)_dropped);
  out.print(tmp);
#if PACKET_LOG_RING_SIZE > 0
  for (int i = 0; i < _count; i++) {
    const Entry& e = _entries[(_head + i) % PACKET_LOG_RING_SIZE];
    sprintf(tmp, "%lu %04x-%lu %s ", (unsigned long)e.timestamp, _boot_id, (unsigned long)e.seq, KIND_NAMES[e.kind]);
    out.print(tmp);
    if (e.kind == RX) {
      int snr = e.snr_x4 < 0 ? -e.snr_x4 : e.snr_x4;
      sprintf(tmp, "%s%d.%02d %d %d ", e.snr_x4 < 0 ? "-" : "", snr / 4, (snr % 4) * 25, e.rssi, e.score);
      out.print(tmp);
    }
    printHexBytes(out, e.raw, e.len);
    out.print('\n');
  }
#endif
}

int PacketLogRing::clearOlderThan(uint32_t now, uint32_t secs, int* kept) {
  int removed = 0;
#if PACKET_LOG_RING_SIZE > 0
  // entries are in arrival order, so drop from the oldest end until one is young enough
  while (_count > 0) {
    const Entry& e = _entries[_head];
    if (now < e.timestamp || now - e.timestamp <= secs) break;
    _head = (_head + 1) % PACKET_LOG_RING_SIZE;
    _count--;
    removed++;
  }
#endif
  *kept = _count;
  return removed;
}
