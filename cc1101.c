#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "cc1101.h"
#include "config.h"

// Strobe commands
#define CC1100_SRES    0x30
#define CC1100_SCAL    0x33
#define CC1100_SRX     0x34
#define CC1100_STX     0x35
#define CC1100_SIDLE   0x36

// Bit fields in the chip status byte
#define CC1100_STATUS_STATE_BM   0x70

// Chip states
#define CC1100_STATE_IDLE   0x00
#define CC1100_STATE_RX     0x10
#define CC1100_STATE_TX     0x20

// CC1101 register settings
static const uint8_t PROGMEM CC_REGISTER_VALUES[] = {
  0x00, 0x0B,  // CCx_IOCFG2 (Serial Clock. Synchronous to the data in synchronous serial mode.)
  0x01, 0x2E,  // CCx_IOCFG1
  0x02, 0x0C,  // CCx_IOCFG0 (Serial Synchronous Data Output. Used for synchronous serial mode.)
  0x07, 0x00,  // CCx_PKTCTRL1
  0x08, 0x12,  // CCx_PKTCTRL0 (Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins)
  0x0B, 0x06,  // CCx_FSCTRL1
  0x0D, 0x21,  // CCx_FREQ2
  0x0E, 0x65,  // CCx_FREQ1
  0x0F, 0x6C,  // CCx_FREQ0
  0x10, 0x6A,  // CCx_MDMCFG4
  0x11, 0x83,  // CCx_MDMCFG3 (DRATE_M=131 data rate=38,383.4838867Hz)
  0x12, 0x10,  // CCx_MDMCFG2 (GFSK No Sync Word / Preamable)
  0x13, 0x22,  // CCx_MDMCFG1 (CHANSPC_E=2 NUM_PREAMBLE=4 FEC_EN=0)
  0x14, 0xF8,  // CCx_MDMCFG0
  0x15, 0x50,  // CCx_DEVIATN
  0x16, 0x07,  // CCx_MCSM2
  0x17, 0x30,  // CCx_MCSM1 (0x30=110000 defaults to idle for RX,TX CCA_MODE=11  If RSSI below threshold unless currently receiving a packet)
  0x18, 0x18,  // CCx_MCSM0 (0x18=11000 FS_AUTOCAL=1 When going from IDLE to RX or TX)
  0x19, 0x16,  // CCx_FOCCFG
  0x1B, 0x43,  // CCx_AGCCTRL2
  0x1C, 0x40,  // CCx_AGCCTRL1
  0x1D, 0x91,  // CCx_AGCCTRL0
  0x23, 0xE9,  // CCx_FSCAL3
  0x24, 0x2A,  // CCx_FSCAL2
  0x25, 0x00,  // CCx_FSCAL1
  0x26, 0x1F,  // CCx_FSCAL0
  0x29, 0x59,  // CCx_FSTEST
  0x2C, 0x81,  // CCx_TEST2
  0x2D, 0x35,  // CCx_TEST1
  0x2E, 0x09,  // CCx_TEST0
  0x3E, 0xC0   // CCx_PATABLE
};

// Radio states
#define RS_RX            0
#define RS_TX            1
#define RS_CHANGE_TO_TX  2
#define RS_CHANGE_TO_RX  3

static accept_bit_fn accept_bit;
static request_bit_fn request_bit;

static volatile uint8_t radio_state;

static void spi_init(void) {
  SPI_PORT |= (1 << SPI_SCLK);
  SPI_DDR  |= ((1 << SPI_MOSI) | (1 << SPI_SCLK) | (1 << SPI_SS));
  SPI_DDR  &= ~(1 << SPI_MISO);

  SPCR  = (1 << MSTR) | (1 << SPE) | (1 << SPR1) | (1 << SPR0);
  // SPSR |= (1 << SPI2X);
}

static inline uint8_t spi_send(uint8_t data) {
  SPDR = data;
  while (!(SPSR & (1 << SPIF)));
  return SPDR;
}

static inline void spi_deassert(void) {
  SPI_PORT |= (1 << SPI_SS);
}

static inline void spi_assert(void) {
  SPI_PORT &= ~(1 << SPI_SS);
}

static uint8_t spi_strobe(uint8_t b) {
  uint8_t result;
  spi_assert();
  while (SPI_PORT & (1 << SPI_MISO));
  result = spi_send(b);
  while (SPI_PORT & (1 << SPI_MISO));
  spi_deassert();
  return result;
}

static void receive_bit(void) {
  uint8_t bit = (GDO0_DATA_IN >> GDO0_DATA_PIN) & 1;
  if (accept_bit(bit) != 0) {
    // Non-0 is a request to switch to transmit mode. This is called from interrupt
    // handlers. Actual switch is delayed until main loop.
    radio_state = RS_CHANGE_TO_TX;
  }
}

static void send_bit(void) {
  uint8_t bit = request_bit();

  // If something other than 0 or 1 is returned, switch to receive mode. The radio
  // will clock in whatever was set on the data pin last, and will transmit noise,
  // but that doesn't matter.
  if (bit == 0) {
    GDO0_DATA_PORT &= ~(1 << GDO0_DATA_PIN);
  } else if (bit == 1) {
    GDO0_DATA_PORT |= (1 << GDO0_DATA_PIN);
  } else {
    // This is called from interrupt handlers. Actual switch is delayed until main loop.
    radio_state = RS_CHANGE_TO_RX;
  }
}

ISR(GDO2_CLK_INTVECT) {
  if (radio_state == RS_RX) {
    receive_bit();
  } else if (radio_state == RS_TX) {
    send_bit();
  }
}

static void cc_enter_rx_mode(void) {
  EIMSK &= ~(1 << GDO2_CLK_INT);            // Disable interrupts

  while ((spi_strobe(CC1100_SIDLE) & CC1100_STATUS_STATE_BM) != CC1100_STATE_IDLE);
  while ((spi_strobe(CC1100_SRX) & CC1100_STATUS_STATE_BM) != CC1100_STATE_RX);

  radio_state = RS_RX;

  GDO0_DATA_DDR &= ~(1 << GDO0_DATA_PIN);   // Set data pin for input
  EICRA |= (1 << GDO2_CLK_INT_ISCn1);       // Set falling edge
  EICRA &= ~(1 << GDO2_CLK_INT_ISCn0);      //   ...
  EIMSK |= (1 << GDO2_CLK_INT);             // Enable interrupts
}

static void cc_enter_tx_mode(void) {
  EIMSK &= ~(1 << GDO2_CLK_INT);            // Disable interrupts

  while ((spi_strobe(CC1100_SIDLE) & CC1100_STATUS_STATE_BM) != CC1100_STATE_IDLE);
  while ((spi_strobe(CC1100_STX) & CC1100_STATUS_STATE_BM) != CC1100_STATE_TX);

  radio_state = RS_TX;

  GDO0_DATA_DDR |= (1 << GDO0_DATA_PIN);    // Set data pin for output
  EICRA |= (1 << GDO2_CLK_INT_ISCn1);       // Set rising edge
  EICRA |= (1 << GDO2_CLK_INT_ISCn0);       //   ...
  EIMSK |= (1 << GDO2_CLK_INT);             // Enable interrupts
}

static uint8_t cc_write(uint8_t addr, uint8_t b) {
  uint8_t result;

  spi_assert();
  while (SPI_PORT & (1 << SPI_MISO));

  spi_send(addr);
  result = spi_send(b);

  spi_deassert();
  return result;
}

void cc_init(accept_bit_fn a, request_bit_fn r) {
  accept_bit = a;
  request_bit = r;

  spi_init();

  spi_deassert();
  _delay_us(1);

  spi_assert();
  _delay_us(10);

  spi_deassert();
  _delay_us(41);

  spi_strobe(CC1100_SRES);
  spi_strobe(CC1100_SCAL);

  for (uint8_t i = 0; i < sizeof(CC_REGISTER_VALUES); ) {
    uint8_t reg = pgm_read_byte(&CC_REGISTER_VALUES[i++]);
    uint8_t val = pgm_read_byte(&CC_REGISTER_VALUES[i++]);
    cc_write(reg, val);
  }

  cc_enter_rx_mode();
}

void cc_work(void) {
  if (radio_state == RS_CHANGE_TO_RX) {
    cc_enter_rx_mode();
  } else if (radio_state == RS_CHANGE_TO_TX) {
    cc_enter_tx_mode();
  }
}
