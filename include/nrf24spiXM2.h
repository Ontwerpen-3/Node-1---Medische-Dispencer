/*!
 *  \file    nrf24spiXM2.h
 *  \author  Wim Dolman (<a href="mailto:w.e.dolman@hva.nl">w.e.dolman@hva.nl</a>)
 *  \date    18-02-2016
 *  \version 1.0
 *
 *  \brief   Header file SPI driver for Nordic NRF24L01p on Xmegaboard-version2
 *
 *  \details This file contains the definitions and prototypes for interfacing
 *           a Nordic NRF24L01p on Xmegaboard-version2 from july 2015.
 *
 *           This driver uses UARTC0 as SPI-interface. The baudrate is the maximum
 *           rate. This is 8 MHz @ Fcpu 32MHz. It doesn't exceed the maximum
 *           speed 10 MHz of the NRF24L01p.
 *
 *
 *           CT 2023-12-14 The PORTs and PINs are now configured in a separate
 *                         header file: nrf24_pindef.h
 *                         See this header file for the actual pin locations.
 *
 *           The interface exists of six signals:
 *               <TABLE>
 *               <TR>
 *                   <TH> Nordic NRF24L01p        </TH>
 *                   <TH> Xmegaboard-version2 or external module </TH>
 *               </TR>
 *               <TR><TD> IRQ   </TD><TD> interrupt request </TD></TR>
 *               <TR><TD> CE    </TD><TD> chip enable       </TD></TR>
 *               <TR><TD> CSN   </TD><TD> SPI slave select  </TD></TR>
 *               <TR><TD> SCK   </TD><TD> SPI clock         </TD></TR>
 *               <TR><TD> MOSI  </TD><TD> SPI MOSI          </TD></TR>
 *               <TR><TD> MISO  </TD><TD> SPI MISO          </TD></TR>
 *               </TABLE>
 *
 *           More information can be found in chapter 8 of the NRF24L01p datasheet.
 */

#ifndef __nrf24spiXM2_H__
#define __nrf24spiXM2_H__

#ifndef F_CPU
#define F_CPU 32000000UL
#endif

#include "nrf24_pindef.h"

#include <avr/io.h>
#include <util/delay.h>

#define NRF_SELECT    0            //!< Spi slave selected
#define NRF_DESELECT  1            //!< Spi slave deselected
#define NRF_ENABLE    1            //!< NRF chip enable
#define NRF_DISABLE   0            //!< NRF chip disable

void     nrfspiInit(void);
uint8_t  nrfspiTransfer(uint8_t iData);

/*! \brief Set chip select
 *
 *  \param bSelected  NRF_SELECT selects SPI bus,
 *                    NRF_DESELECT deselect SPI bus
 *
 *  \return void
 */
inline void nrfCSn(uint8_t bSelected)
{
  if      (bSelected == NRF_DESELECT)  NRF24_CSN_PORT.OUTSET = NRF24_CSN_PIN;
  else if (bSelected == NRF_SELECT)    NRF24_CSN_PORT.OUTCLR = NRF24_CSN_PIN;
}

/*! \brief Set chip enable
 *
 *  \param   bEnabled  NRF_ENABLE enables transmission of Nordic chip
 *                     NRF_DISABLE disables transmission of Nordic chip
 *
 *  \details Level NRF_ENABLE (high) starts transmission and NRF_ENABLE (low)
 *           puts Nordic LOW in standby.
 *
 *  \return  void
 */
inline void nrfCE(uint8_t bEnabled)
{
  if      (bEnabled == NRF_ENABLE)   NRF24_CE_PORT.OUTSET = NRF24_CE_PIN;
  else if (bEnabled == NRF_DISABLE)  NRF24_CE_PORT.OUTCLR = NRF24_CE_PIN;
}

#endif
