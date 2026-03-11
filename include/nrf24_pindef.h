/*!
 * \file    nrf24_pindef.h
 * \author  Caspar Treijtel (<a href="mailto:c.treijtel@hva.nl">c.treijtel@hva.nl</a>)
 * \date    2023-12-14
 * \version 1.0
 *
 * \brief   Definition of pin locations and usart device for usage in the
 *          NRF24L01p library written by Wim Dolman.
 *
 * \details The interface exists of six signals. The pin mappings are defined
 *          according to the selection of one of three options, listed below:
 *
 *          OPT1 : Onboard nRF24L01+ chip on the HvA-xmega board, here the
 *                 USARTC0 device is used for communication with the nrf chip.
 *          OPT2 : External nRF24L01+ module with the HvA-xmega (light)
 *                 board and USARTD0 used for communication with the nrf24
 *                 chip
 *          OPT3 : External nRF24L01+ module with the HvA-xmega (light)
 *                 board and USARTD1 used for communication with the nrf24
 *                 chip
 *
 *          OPTION                       OPT1   | OPT2 | OPT3
 *          Nordic NRF24L01p            onboard | ext. | ext.
 *          IRQ   - interrupt request     PF6   |  PD5 |  PB1
 *          CE    - chip enable           PF7   |  PD6 |  PB0
 *          CSN   - SPI slave select      PF5   |  PD4 |  PB2
 *          SCK   - SPI clock             PC1   |  PD1 |  PD5
 *          MOSI  - SPI MOSI              PC3   |  PD3 |  PD7
 *          MISO  - SPI MISO              PC2   |  PD2 |  PD6
 *
 *   +----------------------------------------------------------------+
 *   |                                             ________           |
 *   |   A                                     IRQ | #  # | MISO      |
 *   |   N         nRF24L01+                       |      |           |
 *   |   T         external                   MOSI | #  # | SCK       |
 *   |   E          module                         |      |           |
 *   |   N                                     CSN | #  # | CE        |
 *   |   N        (top view)                       |      |           |
 *   |   A                                     VCC | #  # | GND       |
 *   |                                             --------           |
 *   +----------------------------------------------------------------+
 */
#ifndef NRF24_PINDEF_H_
#define NRF24_PINDEF_H_


/* SELECT THE DESIRED OPTION BELOW BY (UN)COMMENTING LINES */
/* OPT1 : Onboard nRF24L01+ chip on the HvA-xmega board */
#define INTMOD_USARTC0

/* OPT2 : External nRF24L01+ module, USARTD0 */
// #define EXTMOD_USARTD0

/* OPT3 : External nRF24L01+ module, USARTD1 */
//#define EXTMOD_USARTD1
/* ~SELECT */


#ifdef EXTMOD_USARTD0
#define NRF24_SCK_PORT  PORTD
#define NRF24_SCK_PIN   PIN1_bm
#define NRF24_MISO_PORT PORTD
#define NRF24_MISO_PIN  PIN2_bm
#define NRF24_MOSI_PORT PORTD
#define NRF24_MOSI_PIN  PIN3_bm
#define NRF24_CSN_PORT  PORTD
#define NRF24_CSN_PIN   PIN4_bm
#define NRF24_IRQ_PORT  PORTD
#define NRF24_IRQ_PIN   PIN5_bm
#define NRF24_IRQ_CTRL  PIN5CTRL
#define NRF24_IRQ_VEC   PORTD_INT0_vect
#define NRF24_CE_PORT   PORTD
#define NRF24_CE_PIN    PIN6_bm
#define NRF24_USART     USARTD0
#endif /* EXTMOD_USARTD0 */

#ifdef EXTMOD_USARTD1
#define NRF24_SCK_PORT  PORTD
#define NRF24_SCK_PIN   PIN5_bm
#define NRF24_MISO_PORT PORTD
#define NRF24_MISO_PIN  PIN6_bm
#define NRF24_MOSI_PORT PORTD
#define NRF24_MOSI_PIN  PIN7_bm
#define NRF24_CSN_PORT  PORTB
#define NRF24_CSN_PIN   PIN2_bm
#define NRF24_IRQ_PORT  PORTB
#define NRF24_IRQ_PIN   PIN1_bm
#define NRF24_IRQ_CTRL  PIN1CTRL
#define NRF24_IRQ_VEC   PORTB_INT0_vect
#define NRF24_CE_PORT   PORTB
#define NRF24_CE_PIN    PIN0_bm
#define NRF24_USART     USARTD1
#endif /* EXTMOD_USARTD1 */

#ifdef INTMOD_USARTC0
#define NRF24_SCK_PORT  PORTC
#define NRF24_SCK_PIN   PIN1_bm
#define NRF24_MISO_PORT PORTC
#define NRF24_MISO_PIN  PIN2_bm
#define NRF24_MOSI_PORT PORTC
#define NRF24_MOSI_PIN  PIN3_bm
#define NRF24_CSN_PORT  PORTF
#define NRF24_CSN_PIN   PIN5_bm
#define NRF24_IRQ_PORT  PORTF
#define NRF24_IRQ_PIN   PIN6_bm
#define NRF24_IRQ_CTRL  PIN6CTRL
#define NRF24_IRQ_VEC   PORTF_INT0_vect
#define NRF24_CE_PORT   PORTF
#define NRF24_CE_PIN    PIN7_bm
#define NRF24_USART     USARTC0
#endif /* INTMOD_USARTC0 */

#endif /* NRF24_PINDEF_H_ */
