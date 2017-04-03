#ifndef _CONFIG_H_
#  error "Include config.h instead of this file"
#endif

#ifndef _HW_ARDUINO_H_
#define _HW_ARDUINO_H_

#define UART_BAUD_RATE   25  // 76800 @ 16MHz
#define RINGBUF_SIZE    128

// WARNING: THESE ARE JUST PLACEHOLDER VALUES TO GET COMPILATION WORKING,
// THEY ARE NOT EXPECTED TO WORK ON REAL HARDWARE

// SPI port defs
#define SPI_PORT    PORTB
#define SPI_DDR     DDRB
#define SPI_SS      2
#define SPI_MOSI    3
#define SPI_MISO    4
#define SPI_SCLK    5

// Connection to CC1101 GDO2
#define GDO2_CLK_PIN         1
#define GDO2_CLK_INT         INT1
#define GDO2_CLK_INTVECT     INT1_vect
#define GDO2_CLK_INT_ISCn0   ISC10
#define GDO2_CLK_INT_ISCn1   ISC11

// Connection to CC1101 GDO0
#define GDO0_DATA_DDR     DDRD
#define GDO0_DATA_PORT    PORTD
#define GDO0_DATA_PIN     2
#define GDO0_DATA_IN      PIND

// TTY USART
#define TTY_UDRE_VECT   USART_UDRE_vect
#define TTY_RX_VECT     USART_RX_vect

#endif
