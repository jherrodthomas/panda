#pragma once

// LionDriver TSR-410 / SWSR-410: CRC-32C (Castagnoli) for the SPI link (WP-S-05 §4.1).
// Polynomial 0x1EDC6F41, processed reflected (0x82F63B78), initial value 0xFFFFFFFF, final XOR 0xFFFFFFFF.
// Check value over the ASCII bytes "123456789": 0xE3069283.
// Computed four bits at a time from a 16-entry table (64 bytes of flash).

#define CRC32C_INIT 0xFFFFFFFFU
#define CRC32C_XOROUT 0xFFFFFFFFU

// Continue a CRC-32C over len bytes. Start with CRC32C_INIT and XOR the result with CRC32C_XOROUT at the end.
static uint32_t crc32c_update(uint32_t crc, const uint8_t *dat, uint32_t len) {
  static const uint32_t crc32c_nibble_table[16] = {
    0x00000000U, 0x105EC76FU, 0x20BD8EDEU, 0x30E349B1U, 0x417B1DBCU, 0x5125DAD3U, 0x61C69362U, 0x7198540DU,
    0x82F63B78U, 0x92A8FC17U, 0xA24BB5A6U, 0xB21572C9U, 0xC38D26C4U, 0xD3D3E1ABU, 0xE330A81AU, 0xF36E6F75U,
  };
  uint32_t c = crc;
  for (uint32_t i = 0U; i < len; i++) {
    c ^= (uint32_t)dat[i];
    c = (c >> 4U) ^ crc32c_nibble_table[c & 0xFU];
    c = (c >> 4U) ^ crc32c_nibble_table[c & 0xFU];
  }
  return c;
}
