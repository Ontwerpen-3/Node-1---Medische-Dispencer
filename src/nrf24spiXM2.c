/*!
 *  \file    nrf24spiXM2.c
 *  \author  Wim Dolman (<a href="mailto:w.e.dolman@hva.nl">w.e.dolman@hva.nl</a>)
 *  \date    18-02-2016
 *  \version 1.0
 *
 *  \brief   Spi driver for Nordic NRF24L01p on HvA Xmegaboard-version2
 *
 *  \details This file contains the routines for interfacing a Nordic NRF24L01p
 *           on HvA Xmegaboard-version2 from july 2015.
 *
 *           This driver uses UARTD0 as SPI-interface. The baudrate is the maximum
 *           rate. This is 8 MHz with Fcpu 32MHz and doesn't exceed the maximum
 *           speed 10 MHz from the NRF24L01p.
 *
 *
 *           CT 2023-12-14 The PORTs and PINs are now configured in a separate
 *                         header file: nrf24_pindef.h
 *                         See this header file for the actual pin locations.
 *
 *           The interface exists of six signals:
 *           -   Nordic NRF24L01p           | Xmegaboard-version2 or external module
 *           -   IRQ   - interrupt request
 *           -   CE    - chip enable
 *           -   CSN   - SPI slave select
 *           -   SCK   - SPI clock
 *           -   MOSI  - SPI MOSI
 *           -   MISO  - SPI MISO
 */
#include "nrf24spiXM2.h"

/*! \brief   Initialization of SPI
 *
 *  \details This routines has no parameters. It Initializes UARTD0 as SPI
 *           and the signals IRQ and CE
 *
 *  \return void
 */

void nrfspiInit(void)
{
  NRF24_MOSI_PORT.DIRSET = NRF24_MOSI_PIN;  // MOSI
  NRF24_MISO_PORT.DIRCLR = NRF24_MISO_PIN;  // MISO
  NRF24_SCK_PORT.DIRSET  = NRF24_SCK_PIN;   // SCK
  NRF24_CSN_PORT.DIRSET  = NRF24_CSN_PIN;   // CSN
  NRF24_IRQ_PORT.DIRCLR  = NRF24_IRQ_PIN;   // IRQ
  NRF24_CE_PORT.DIRSET   = NRF24_CE_PIN;    // CE

  NRF24_USART.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
  NRF24_USART.CTRLC = USART_CMODE_MSPI_gc;

  NRF24_USART.BAUDCTRLB = 0;
  NRF24_USART.BAUDCTRLA = 1;   // F_CPU/(2*(BSEL+1))  is 8MHz on 32MHz CPU
}

/*! \brief SPI transfer
 *
 *  \param   iData    data byte send to the slave
 *
 *  \details This function send a byte IData to register to the slave,
 *           while data from the slave is received into the DATA register.
 *
 *           In this case, for the nrf24L01p, the value of status-register
 *           is shifted out
 *
 *  \return  Data received from slave (status of the nrf24L01p)
 */
uint8_t nrfspiTransfer(uint8_t iData)
{
  NRF24_USART.DATA = iData;
  while( !(NRF24_USART.STATUS & USART_TXCIF_bm) );
  NRF24_USART.STATUS |= USART_TXCIF_bm;

  return NRF24_USART.DATA;
}


