#pragma once

#include "board/drivers/drivers.h"
#include "board/crc32c.h"

// H7 DMA2 located in D2 domain, so we need to use SRAM1/SRAM2
#ifdef STM32H7
__attribute__((section(".sram12"))) uint8_t spi_buf_rx[SPI_BUF_SIZE];
__attribute__((section(".sram12"))) uint8_t spi_buf_tx[SPI_BUF_SIZE];
#else
uint8_t spi_buf_rx[SPI_BUF_SIZE];
uint8_t spi_buf_tx[SPI_BUF_SIZE];
#endif

#define SPI_SYNC_BYTE 0x5AU
#define SPI_HACK 0x79U
#define SPI_DACK 0x85U
#define SPI_NACK 0x1FU
// LionDriver SWSR-410a: transfer refused because its sequence counter is out of step; the host restarts its counter
#define SPI_SEQ_NACK 0x2EU

// LionDriver SPI protocol version 3 (TSR-410, 411, 412; WP-S-05 §4.1). Version 2 used an 8-bit XOR seeded 0xAB.
#define SPI_PROTOCOL_VERSION 3U

// SPI states
enum {
  SPI_STATE_HEADER,
  SPI_STATE_HEADER_ACK,
  SPI_STATE_HEADER_NACK,
  SPI_STATE_DATA_RX,
  SPI_STATE_DATA_RX_ACK,
  SPI_STATE_DATA_TX
};

uint16_t spi_error_count = 0;

// Header, SoC -> MCU: sync, endpoint, MOSI payload length (2, LE), max MISO payload length (2, LE), CRC-8.
// The header stays 7 bytes so the "VERSION" request (7 bytes) is still recognised by every protocol version.
#define SPI_HEADER_SIZE 7U
#define SPI_HEADER_CRC_LEN 6U
#define SPI_HEADER_CRC_POLY 0x2FU
// Data, SoC -> MCU: sequence counter, control-frame index, payload, CRC-32C (4, LE) over header bytes 0-5,
// sequence counter, control-frame index and payload.
#define SPI_DATA_PREFIX_SIZE 2U
#define SPI_CRC_SIZE 4U
// Response, MCU -> SoC: DACK, payload length (2, LE), echoed sequence counter, payload, CRC-32C (4, LE) over all of it.
#define SPI_RESP_HEADER_SIZE 4U
// TSR-412: the data phase in either direction is at most 2047 bytes, checked before any DMA is started
#define SPI_MAX_DATA_PHASE 2047U
#define SPI_MAX_MOSI_LEN (SPI_MAX_DATA_PHASE - SPI_DATA_PREFIX_SIZE - SPI_CRC_SIZE)
#define SPI_MAX_MISO_LEN (SPI_MAX_DATA_PHASE - SPI_RESP_HEADER_SIZE - SPI_CRC_SIZE)

// sequence counter check results (SWSR-410a)
#define SPI_SEQ_NEW 0U
#define SPI_SEQ_REPEAT 1U
#define SPI_SEQ_BAD 2U

// low level SPI prototypes
void llspi_init(void);
void llspi_mosi_dma(uint8_t *addr, int len);
void llspi_miso_dma(const uint8_t *addr, int len);

static uint8_t spi_state = SPI_STATE_HEADER;
static uint16_t spi_data_len_mosi;
static uint16_t spi_data_len_miso;
static bool spi_can_tx_ready = false;
static bool spi_seq_valid = false;
static uint8_t spi_seq_last = 0U;
static const unsigned char version_text[] = "VERSION";

static uint16_t spi_version_packet(uint8_t *out) {
  // this protocol version request is a stable portion of
  // the panda's SPI protocol. its contents match that of the
  // panda USB descriptors and are sufficent to list/enumerate
  // a panda, determine panda type, and bootstub status.

  // the response is:
  // VERSION + 2 byte data length + data + CRC8

  // echo "VERSION"
  (void)memcpy(out, version_text, 7);

  // write response
  uint16_t data_len = 0;
  uint16_t data_pos = 7U + 2U;

  // write serial
  (void)memcpy(&out[data_pos], ((uint8_t *)UID_BASE), 12);
  data_len += 12U;

  // HW type
  out[data_pos + data_len] = hw_type;
  data_len += 1U;

  // bootstub
  out[data_pos + data_len] = USB_PID & 0xFFU;
  data_len += 1U;

  // SPI protocol version
  out[data_pos + data_len] = SPI_PROTOCOL_VERSION;
  data_len += 1U;

  // data length
  out[7] = data_len & 0xFFU;
  out[8] = (data_len >> 8) & 0xFFU;

  // CRC8
  uint16_t resp_len = data_pos + data_len;
  out[resp_len] = crc_checksum(out, resp_len, 0xD5U);
  resp_len += 1U;

  return resp_len;
}

void spi_init(void) {
  // platform init
  llspi_init();

  // Start the first packet!
  spi_state = SPI_STATE_HEADER;
  llspi_mosi_dma(spi_buf_rx, SPI_HEADER_SIZE);
}

// SWSR-410, SWSR-412: sync byte, header CRC-8 and both lengths within the data-phase bound
static bool spi_header_valid(void) {
  const bool crc_ok = crc_checksum(spi_buf_rx, (int)SPI_HEADER_CRC_LEN, SPI_HEADER_CRC_POLY) == spi_buf_rx[SPI_HEADER_CRC_LEN];
  const bool len_ok = (spi_data_len_mosi <= SPI_MAX_MOSI_LEN) && (spi_data_len_miso <= SPI_MAX_MISO_LEN);
  return (spi_buf_rx[0] == SPI_SYNC_BYTE) && crc_ok && len_ok;
}

static uint32_t spi_read_le32(const uint8_t *p) {
  return ((uint32_t)p[0]) | ((uint32_t)p[1] << 8U) | ((uint32_t)p[2] << 16U) | ((uint32_t)p[3] << 24U);
}

static void spi_write_le32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFFU);
  p[1] = (uint8_t)((v >> 8U) & 0xFFU);
  p[2] = (uint8_t)((v >> 16U) & 0xFFU);
  p[3] = (uint8_t)((v >> 24U) & 0xFFU);
}

// SWSR-410: CRC-32C over header bytes 0-5, sequence counter, control-frame index and payload
static bool spi_data_crc_valid(void) {
  const uint32_t covered = SPI_DATA_PREFIX_SIZE + (uint32_t)spi_data_len_mosi;
  uint32_t crc = crc32c_update(CRC32C_INIT, spi_buf_rx, SPI_HEADER_CRC_LEN);
  crc = crc32c_update(crc, &spi_buf_rx[SPI_HEADER_SIZE], covered) ^ CRC32C_XOROUT;
  return crc == spi_read_le32(&spi_buf_rx[SPI_HEADER_SIZE + covered]);
}

static uint8_t spi_seq_next(uint8_t seq) {
  // the host never sends 0 after the first transfer of its session: 255 is followed by 1
  return (seq == 255U) ? 1U : (uint8_t)(seq + 1U);
}

// SWSR-410a: accept a counter step of 1 or 2 from the last executed transfer. 0 starts a host session and is
// always accepted (the host sends it on its first transfer and after an SPI_SEQ_NACK). The last executed counter
// again is a repetition: not executed again. Anything else is out of step.
static uint8_t spi_seq_check(uint8_t seq) {
  uint8_t ret = SPI_SEQ_BAD;
  if ((seq == 0U) || !spi_seq_valid) {
    ret = SPI_SEQ_NEW;
  } else if (seq == spi_seq_last) {
    ret = SPI_SEQ_REPEAT;
  } else if ((seq == spi_seq_next(spi_seq_last)) || (seq == spi_seq_next(spi_seq_next(spi_seq_last)))) {
    ret = SPI_SEQ_NEW;
  } else {
    ret = SPI_SEQ_BAD;
  }
  return ret;
}

// Pass the payload to the endpoint's handler. Returns true to acknowledge; the response payload goes to
// spi_buf_tx[SPI_RESP_HEADER_SIZE] and its length to *response_len.
static bool spi_dispatch(uint8_t endpoint, uint16_t *response_len) {
  bool response_ack = false;
  const uint8_t *payload = &spi_buf_rx[SPI_HEADER_SIZE + SPI_DATA_PREFIX_SIZE];
  uint8_t *resp_payload = &spi_buf_tx[SPI_RESP_HEADER_SIZE];
  if (endpoint == 0U) {
    if (spi_data_len_mosi >= sizeof(ControlPacket_t)) {
      ControlPacket_t ctrl = {0};
      (void)memcpy((uint8_t*)&ctrl, payload, sizeof(ControlPacket_t));
      *response_len = comms_control_handler(&ctrl, resp_payload);
      response_ack = true;
    } else {
      print("SPI: insufficient data for control handler\n");
    }
  } else if ((endpoint == 1U) || (endpoint == 0x81U)) {
    if (spi_data_len_mosi == 0U) {
      *response_len = comms_can_read(resp_payload, spi_data_len_miso);
      response_ack = true;
    } else {
      print("SPI: did not expect data for can_read\n");
    }
  } else if (endpoint == 2U) {
    comms_endpoint2_write(payload, spi_data_len_mosi);
    response_ack = true;
  } else if (endpoint == 3U) {
    if ((spi_data_len_mosi > 0U) && spi_can_tx_ready) {
      spi_can_tx_ready = false;
      comms_can_write(payload, spi_data_len_mosi);
      response_ack = true;
    } else {
      print("SPI: CAN NACK\n");
    }
  } else if (endpoint == 0xABU) {
    // test endpoint: mimics panda -> device transfer
    *response_len = spi_data_len_miso;
    response_ack = true;
  } else if (endpoint == 0xACU) {
    // test endpoint: mimics device -> panda transfer (with NACK)
    response_ack = false;
  } else {
    print("SPI: unexpected endpoint"); puth(endpoint); print("\n");
  }
  return response_ack;
}

// Build the data response in spi_buf_tx: DACK, length, echoed sequence counter, payload, CRC-32C. Returns its size.
static uint16_t spi_build_dack(uint16_t payload_len, uint8_t seq) {
  const uint32_t covered = SPI_RESP_HEADER_SIZE + (uint32_t)payload_len;
  spi_buf_tx[0] = SPI_DACK;
  spi_buf_tx[1] = payload_len & 0xFFU;
  spi_buf_tx[2] = (payload_len >> 8) & 0xFFU;
  spi_buf_tx[3] = seq;
  spi_write_le32(&spi_buf_tx[covered], crc32c_update(CRC32C_INIT, spi_buf_tx, covered) ^ CRC32C_XOROUT);
  return (uint16_t)(covered + SPI_CRC_SIZE);
}

static uint16_t spi_build_nack(uint8_t code) {
  spi_buf_tx[0] = code;
  return 1U;
}

// Data phase received: check CRC and counter, run the endpoint, build the response. Sets *fault for a transfer
// that counts as an SPI error (SWSR-413). Returns the response size; *next_state is the state after sending it.
static uint16_t spi_handle_data(uint8_t endpoint, uint8_t *next_state, bool *fault) {
  uint16_t response_len = 0U;
  const uint8_t seq = spi_buf_rx[SPI_HEADER_SIZE];
  *next_state = SPI_STATE_HEADER_NACK;
  if (!spi_data_crc_valid()) {
    *fault = true;
    response_len = spi_build_nack(SPI_NACK);
  } else {
    const uint8_t seq_result = spi_seq_check(seq);
    if (seq_result == SPI_SEQ_REPEAT) {
      // already executed: acknowledge without running it again, and count it
      *fault = true;
      response_len = spi_build_dack(0U, seq);
      *next_state = SPI_STATE_DATA_TX;
    } else if (seq_result == SPI_SEQ_BAD) {
      *fault = true;
      response_len = spi_build_nack(SPI_SEQ_NACK);
    } else {
      uint16_t payload_len = 0U;
      if (spi_dispatch(endpoint, &payload_len) && (payload_len <= SPI_MAX_MISO_LEN)) {
        spi_seq_last = seq;
        spi_seq_valid = true;
        response_len = spi_build_dack(payload_len, seq);
        *next_state = SPI_STATE_DATA_TX;
      } else {
        response_len = spi_build_nack(SPI_NACK);
      }
    }
  }
  return response_len;
}

void spi_rx_done(void) {
  uint16_t response_len = 0U;
  uint8_t next_rx_state = SPI_STATE_HEADER_NACK;
  bool fault = false;
  static uint8_t spi_endpoint;

  if (memcmp(spi_buf_rx, version_text, 7) == 0) {
    response_len = spi_version_packet(spi_buf_tx);
  } else if (spi_state == SPI_STATE_HEADER) {
    // parse header
    spi_endpoint = spi_buf_rx[1];
    spi_data_len_mosi = (spi_buf_rx[3] << 8) | spi_buf_rx[2];
    spi_data_len_miso = (spi_buf_rx[5] << 8) | spi_buf_rx[4];
    if (spi_header_valid()) {
      // response: ACK and start receiving data portion
      spi_buf_tx[0] = SPI_HACK;
      next_rx_state = SPI_STATE_HEADER_ACK;
      response_len = 1U;
    } else {
      // response: NACK and reset state machine
      fault = true;
      response_len = spi_build_nack(SPI_NACK);
    }
  } else if (spi_state == SPI_STATE_DATA_RX) {
    response_len = spi_handle_data(spi_endpoint, &next_rx_state, &fault);
  } else {
    print("SPI: RX unexpected state: "); puth(spi_state); print("\n");
  }

  // send out response
  if (response_len == 0U) {
    print("SPI: no response\n");
    response_len = spi_build_nack(SPI_NACK);
    next_rx_state = SPI_STATE_HEADER_NACK;
  }
  llspi_miso_dma(spi_buf_tx, response_len);

  spi_state = next_rx_state;
  if (fault) {
    spi_error_count += 1U;
  }
}

void spi_tx_done(bool reset) {
  if ((spi_state == SPI_STATE_HEADER_NACK) || reset) {
    // Reset state
    spi_state = SPI_STATE_HEADER;
    llspi_mosi_dma(spi_buf_rx, SPI_HEADER_SIZE);
  } else if (spi_state == SPI_STATE_HEADER_ACK) {
    // ACK was sent, queue up the RX buf for the counter, frame index, data and CRC (length bounded by TSR-412)
    uint32_t data_phase_len = SPI_DATA_PREFIX_SIZE;
    data_phase_len += spi_data_len_mosi;
    data_phase_len += SPI_CRC_SIZE;
    spi_state = SPI_STATE_DATA_RX;
    llspi_mosi_dma(&spi_buf_rx[SPI_HEADER_SIZE], (int)data_phase_len);
  } else if (spi_state == SPI_STATE_DATA_TX) {
    // Reset state
    spi_state = SPI_STATE_HEADER;
    llspi_mosi_dma(spi_buf_rx, SPI_HEADER_SIZE);
  } else {
    spi_state = SPI_STATE_HEADER;
    llspi_mosi_dma(spi_buf_rx, SPI_HEADER_SIZE);
    print("SPI: TX unexpected state: "); puth(spi_state); print("\n");
  }
}

void can_tx_comms_resume_spi(void) {
  spi_can_tx_ready = true;
}
