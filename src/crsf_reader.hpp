#include <Arduino.h>
#include <array>

/* ─────────── user‑config ─────────── */
constexpr int   RX_PIN     = 16;        // JR‑bay DATA  ➜  GPIO16
constexpr long  BAUD       = 400000;    // EdgeTX External‑RF 400 k (or 420 k)
constexpr bool  INVERT_RX  = true;      // Crossfire/ELRS line idles LOW
/* ──────────────────────────────────── */


class CRSFReader {
 public:
  void begin() {
    Serial2.begin(BAUD, SERIAL_8N1, RX_PIN, -1, INVERT_RX);
  }

  /* Call as often as you like; returns true once per good RC frame   *
   * (≈ 250 Hz) after updating the internal channel array.            */
  bool update() {
    while (Serial2.available()) {
      uint8_t b = Serial2.read();

      switch (state_) {
        case WAIT_START:
          if (b == 0xEE) {
            buf_[0] = b;
            state_  = WAIT_LEN;
          }
          break;

        case WAIT_LEN:
          len_     = b;
          buf_[1]  = b;
          idx_     = 0;
          state_   = COLLECT_BODY;
          break;

        case COLLECT_BODY:
          buf_[2 + idx_] = b;
          if (++idx_ == len_) {             // full body collected
            state_ = WAIT_START;
            if (decode_frame()) return true;
          }
          break;
      }
    }
    return false;
  }

  /* 1‑based index (1 … 16).  Returns 0 on out‑of‑range. */
  uint16_t get_channel(int n) const {
    return (n >= 1 && n <= 16) ? channels_[n - 1] : 0;
  }

 private:
  enum { WAIT_START, WAIT_LEN, COLLECT_BODY } state_ = WAIT_START;

  uint8_t                    buf_[26]{};   // 0xEE + len + 24‑byte body
  uint8_t                    len_  = 0;
  uint8_t                    idx_  = 0;
  std::array<uint16_t, 16>   channels_{};  // raw 0‑2047

  /* CRSF CRC‑8 (polynomial 0xD5, init 0) */
  static uint8_t crc8(const uint8_t* d, uint8_t len) {
    uint8_t crc = 0;
    while (len--) {
      crc ^= *d++;
      for (uint8_t i = 0; i < 8; ++i)
        crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
    }
    return crc;
  }

    bool decode_frame() {
    if (len_ != 0x18 || buf_[2] != 0x16) return false;           // wrong type/len
    if (crc8(buf_ + 2, len_) != 0)        return false;          // CRC fail

    const uint8_t *p = buf_ + 3;          // 22‑byte payload starts here

    for (int ch = 0; ch < 16; ++ch) {
        uint16_t bit_pos   = ch * 11;              // where this channel starts
        uint16_t byte_idx  = bit_pos >> 3;         // /8
        uint8_t  bit_off   = bit_pos & 0x07;       // %8

        /* pull 3 consecutive bytes so we have up to 24 bits to shift from   *
        * (safe because byte_idx+2 ≤ 21 for ch 15)                          */
        uint32_t three     =  p[byte_idx] |
                            (uint32_t)p[byte_idx + 1] << 8 |
                            (uint32_t)p[byte_idx + 2] << 16;

        channels_[ch] = (three >> bit_off) & 0x7FF;   // store 11‑bit value
    }
    return true;           // a complete, CRC‑OK RC_CHANNELS_PACKED frame
    }
};