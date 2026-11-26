// Host harness for board/drivers/spi.h (LionDriver SWSR-410, 410a, 412): the firmware with a simulated SPI4 bus and
// recording endpoint handlers. Used by tests/spi_protocol/test_spi_protocol.py (python/spi.py as the host) and by
// openpilot/selfdrive/pandad/tests/test_pandad_spi_protocol.py (pandad's spi.cc as the host).
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "board/drivers/drivers.h"
#include "board/comms_definitions.h"

static uint8_t fake_uid[12] = {1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U, 12U};
#define UID_BASE fake_uid
uint8_t hw_type = 7U;
#define USB_PID 0xddccU
void print(const char *a) { (void)a; }
void puth(unsigned int i) { (void)i; }

// endpoint handlers: record what the firmware executes
int control_calls = 0;
uint8_t last_request = 0U;
int comms_control_handler(ControlPacket_t *req, uint8_t *resp) {
  control_calls++;
  last_request = req->request;
  int len = (int)req->length;
  for (int i = 0; i < len; i++) { resp[i] = (uint8_t)(req->request + i); }
  return len;
}
int can_read_avail = 0;
int comms_can_read(uint8_t *data, uint32_t max_len) {
  int n = (can_read_avail < (int)max_len) ? can_read_avail : (int)max_len;
  for (int i = 0; i < n; i++) { data[i] = (uint8_t)(0xA0 + i); }
  can_read_avail -= n;
  return n;
}
void comms_endpoint2_write(const uint8_t *data, uint32_t len) { (void)data; (void)len; }
int can_write_calls = 0;
uint32_t can_write_bytes = 0U;
uint8_t can_write_last[4096];
void comms_can_write(const uint8_t *data, uint32_t len) {
  can_write_calls++;
  can_write_bytes += len;
  memcpy(can_write_last, data, len);
  can_tx_comms_resume_spi();  // the firmware resumes when its CAN TX buffer has room again; here at once
}

// simulated SPI4 with its two DMA streams
static uint8_t *rx_addr = NULL; static int rx_len = 0; static int rx_pos = 0;
static const uint8_t *tx_addr = NULL; static int tx_len = 0; static int tx_pos = 0;
int max_mosi_dma = 0;
void llspi_init(void) {}
void llspi_mosi_dma(uint8_t *addr, int len) {
  rx_addr = addr; rx_len = len; rx_pos = 0; tx_len = 0; tx_pos = 0;
  if (len > max_mosi_dma) { max_mosi_dma = len; }
}
void llspi_miso_dma(const uint8_t *addr, int len) { tx_addr = addr; tx_len = len; tx_pos = 0; rx_len = 0; rx_pos = 0; }

#include "board/drivers/spi.h"

// The RX DMA-complete interrupt runs as soon as the last byte is in (spi_rx_done). The end of a response is seen
// by the SPI4 end-of-transfer interrupt when the host releases chip select (spi_tx_done): bytes the host clocks
// after the response in the same transaction are under-run on MISO and are drained, not received.
static bool tx_complete = false;
uint8_t bus_byte(uint8_t mosi) {
  uint8_t miso = 0xcdU;  // SPI4 under-run value
  if (tx_pos < tx_len) {
    miso = tx_addr[tx_pos];
    tx_pos++;
    if (tx_pos == tx_len) { tx_complete = true; }
  } else if (tx_complete) {
  } else if (rx_pos < rx_len) {
    rx_addr[rx_pos] = mosi;
    rx_pos++;
    if (rx_pos == rx_len) { spi_rx_done(); }
  } else {
  }
  return miso;
}
void end_transaction(void) {
  if (tx_complete) {
    tx_complete = false;
    spi_tx_done(false);
  }
}
void tx_ready(void) { can_tx_comms_resume_spi(); }
void tx_full(void) { spi_can_tx_ready = false; }
uint16_t errors(void) { return spi_error_count; }
void reset_mcu(void) {
  spi_seq_valid = false; spi_seq_last = 0U; spi_error_count = 0U; tx_complete = false;
  control_calls = 0; can_write_calls = 0; can_write_bytes = 0U; can_read_avail = 0; max_mosi_dma = 0;
  spi_can_tx_ready = true;
  spi_init();
}
uint32_t crc32c_of(const uint8_t *d, uint32_t len) { return crc32c_update(CRC32C_INIT, d, len) ^ CRC32C_XOROUT; }
int max_mosi_len(void) { return (int)SPI_MAX_MOSI_LEN; }
int max_miso_len(void) { return (int)SPI_MAX_MISO_LEN; }
int protocol_version(void) { return (int)SPI_PROTOCOL_VERSION; }
// control-frame index of the last data phase received (TSR-411 field; not yet checked by the firmware)
uint8_t last_frame_index(void) { return spi_buf_rx[SPI_HEADER_SIZE + 1U]; }
uint32_t written_bytes(void) { return can_write_bytes; }
int written_calls(void) { return can_write_calls; }
int controls(void) { return control_calls; }
void set_can_read_avail(int n) { can_read_avail = n; }
