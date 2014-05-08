/*
 * Copyright (C) 2014, Michele Balistreri
 *
 * Derived from code originally Copyright (C) 2011, Alex Hornung
 *
 * Permission is hereby granted, free of charge, to any person obtaining a 
 * copy of this software and associated documentation files (the "Software"), 
 * to deal in the Software without restriction, including without limitation 
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, 
 * and/or sell copies of the Software, and to permit persons to whom the 
 * Software is furnished to do so, subject to the following conditions: 
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software. 
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL 
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 */

#include "config.h"
#include "MRF24J40.h"

#include <stdlib.h>

#define mrf24j40_spi_preamble() volatile uint8_t tmpIE = mrf24j40_get_ie(); mrf24j40_set_ie(0); mrf24j40_cs_pin(0);
#define mrf24j40_spi_postamble() mrf24j40_cs_pin(1); mrf24j40_set_ie(tmpIE);

void _mrf24j40_write_long_addr(uint16_t addr, uint8_t write) {
  mrf24j40_spi_write(((addr >> 3) & 0x7F) | 0x80);
  mrf24j40_spi_write(((addr << 5) & 0xE0) | (write << 4));
}

void _mrf24j40_write_short_addr(uint8_t addr, uint8_t write) {
  mrf24j40_spi_write(((addr << 1) & 0x7E) | write);
}

uint8_t mrf24j40_read_long_ctrl_reg(uint16_t addr) {
  mrf24j40_spi_preamble();
  _mrf24j40_write_long_addr(addr, 0);
  uint8_t value = mrf24j40_spi_read();
  mrf24j40_spi_postamble();

  return value;
}

uint8_t mrf24j40_read_short_ctrl_reg(uint8_t addr) {
  mrf24j40_spi_preamble();
  _mrf24j40_write_short_addr(addr, 0);
  uint8_t value = mrf24j40_spi_read();
  mrf24j40_spi_postamble();
  return value;
}

void mrf24j40_write_long_ctrl_reg(uint16_t addr, uint8_t value) {
  mrf24j40_spi_preamble();
  _mrf24j40_write_long_addr(addr, 1);
  mrf24j40_spi_write(value);
  mrf24j40_spi_postamble();
}

void mrf24j40_write_short_ctrl_reg(uint8_t addr, uint8_t value) {
  mrf24j40_spi_preamble();
  _mrf24j40_write_short_addr(addr, 1);
  mrf24j40_spi_write(value);
  mrf24j40_spi_postamble();
}

void mrf24j40_ie(void) {
  mrf24j40_write_short_ctrl_reg(MRF24J40_INTCON, ~(TXNIE | RXIE | SECIE));
}

void mrf24j40_pwr_reset(void) {
  mrf24j40_write_short_ctrl_reg(SOFTRST, RSTPWR);
}

void mrf24j40_bb_reset(void) {
  mrf24j40_write_short_ctrl_reg(SOFTRST, RSTBB);
}

void mrf24j40_mac_reset(void) {
  mrf24j40_write_short_ctrl_reg(SOFTRST, RSTMAC);
}

void mrf24j40_rf_reset(void) {
  uint8_t old = mrf24j40_read_short_ctrl_reg(RFCTL);

  mrf24j40_write_short_ctrl_reg(RFCTL, old | RFRST);
  mrf24j40_write_short_ctrl_reg(RFCTL, old & ~RFRST);
  mrf24j40_delay_ms(2);
}

uint8_t mrf24j40_get_pending_frame(void) {
  return (mrf24j40_read_short_ctrl_reg(TXNCON) >> 4) & 0x01;
}

void mrf24j40_rxfifo_flush(void) {
  mrf24j40_write_short_ctrl_reg(RXFLUSH, (mrf24j40_read_short_ctrl_reg(RXFLUSH) | _RXFLUSH));
}

void mrf24j40_set_channel(int16_t ch) {
  mrf24j40_write_long_ctrl_reg(RFCON0, CHANNEL(ch) | RFOPT(0x03));
  mrf24j40_rf_reset();
}

void mrf24j40_set_promiscuous(int16_t crc_check) {
  uint8_t w = NOACKRSP;
  if (!crc_check) {
    w |= ERRPKT;
  } else {
    w |= PROMI;
  }

  mrf24j40_write_short_ctrl_reg(RXMCR, w);
}

void mrf24j40_set_coordinator(void) {
  mrf24j40_write_short_ctrl_reg(RXMCR, mrf24j40_read_short_ctrl_reg(RXMCR) | PANCOORD);
}

void mrf24j40_clear_coordinator(void) {
  mrf24j40_write_short_ctrl_reg(RXMCR, mrf24j40_read_short_ctrl_reg(RXMCR) & ~PANCOORD);
}

void mrf24j40_set_pan(uint8_t *pan) {
  mrf24j40_write_short_ctrl_reg(PANIDL, pan[0]);
  mrf24j40_write_short_ctrl_reg(PANIDH, pan[1]);
}

void mrf24j40_set_short_addr(uint8_t *addr) {
  mrf24j40_write_short_ctrl_reg(SADRL, addr[0]);
  mrf24j40_write_short_ctrl_reg(SADRH, addr[1]);
}

void mrf24j40_set_eui(uint8_t *eui) {
  mrf24j40_write_short_ctrl_reg(EADR0, eui[0]);
  mrf24j40_write_short_ctrl_reg(EADR1, eui[1]);
  mrf24j40_write_short_ctrl_reg(EADR2, eui[2]);
  mrf24j40_write_short_ctrl_reg(EADR3, eui[3]);
  mrf24j40_write_short_ctrl_reg(EADR4, eui[4]);
  mrf24j40_write_short_ctrl_reg(EADR5, eui[5]);
  mrf24j40_write_short_ctrl_reg(EADR6, eui[6]);
  mrf24j40_write_short_ctrl_reg(EADR7, eui[7]);
}

void mrf24j40_set_coordinator_short_addr(uint8_t *addr) {
  mrf24j40_write_short_ctrl_reg(ASSOSADR0, addr[0]);
  mrf24j40_write_short_ctrl_reg(ASSOSADR1, addr[1]);
}

void mrf24j40_set_coordinator_eui(uint8_t *eui) {
  mrf24j40_write_short_ctrl_reg(ASSOEADR0, eui[0]);
  mrf24j40_write_short_ctrl_reg(ASSOEADR1, eui[1]);
  mrf24j40_write_short_ctrl_reg(ASSOEADR2, eui[2]);
  mrf24j40_write_short_ctrl_reg(ASSOEADR3, eui[3]);
  mrf24j40_write_short_ctrl_reg(ASSOEADR4, eui[4]);
  mrf24j40_write_short_ctrl_reg(ASSOEADR5, eui[5]);
  mrf24j40_write_short_ctrl_reg(ASSOEADR6, eui[6]);
  mrf24j40_write_short_ctrl_reg(ASSOEADR7, eui[7]);
}

void mrf24j40_set_key(uint16_t address, uint8_t *key) {
  mrf24j40_spi_preamble();
  _mrf24j40_write_long_addr(address, 1);

  for (int16_t i = 0; i < 16; i++) {
    mrf24j40_spi_write(key[i]);
  }

  mrf24j40_spi_postamble();
}

void mrf24j40_hard_reset() {
  mrf24j40_reset_pin(0);

  mrf24j40_delay_us(192);
  mrf24j40_reset_pin(1);
  mrf24j40_delay_us(192);
}

void mrf24j40_initialize() {
  mrf24j40_cs_pin(1);
  mrf24j40_wake_pin(0);
  
  mrf24j40_hard_reset();
  
  mrf24j40_write_short_ctrl_reg(SOFTRST, (RSTPWR | RSTBB | RSTMAC));

  mrf24j40_delay_us(192);

  mrf24j40_write_short_ctrl_reg(PACON2, FIFOEN | TXONTS(0x18));
  mrf24j40_write_short_ctrl_reg(TXSTBL, RFSTBL(9) | MSIFS(5));
  mrf24j40_write_long_ctrl_reg(RFCON1, VCOOPT(0x01));
  mrf24j40_write_long_ctrl_reg(RFCON2, PLLEN);
  mrf24j40_write_long_ctrl_reg(RFCON6, _20MRECVR);
  mrf24j40_write_long_ctrl_reg(RFCON7, SLPCLKSEL(0x02));
  mrf24j40_write_long_ctrl_reg(RFCON8, RFVCO);
  mrf24j40_write_long_ctrl_reg(SLPCON1, SLPCLKDIV(1) | CLKOUTDIS);

  mrf24j40_write_short_ctrl_reg(BBREG2, CCAMODE(0x02) | CCASTH(0x00));
  mrf24j40_write_short_ctrl_reg(CCAEDTH, 0x60);

  mrf24j40_write_short_ctrl_reg(BBREG6, RSSIMODE2);

  mrf24j40_rxfifo_flush();
  
  mrf24j40_ie();
}

void mrf24j40_sleep(int16_t spi_wake) {
  mrf24j40_write_short_ctrl_reg(WAKECON, IMMWAKE);

  uint8_t r = mrf24j40_read_short_ctrl_reg(SLPACK);

  if (!spi_wake) {
    mrf24j40_wake_pin(0);
    mrf24j40_write_short_ctrl_reg(RXFLUSH, mrf24j40_read_short_ctrl_reg(RXFLUSH) | WAKEPAD | WAKEPOL);
  }

  mrf24j40_pwr_reset();
  mrf24j40_write_short_ctrl_reg(SLPACK, r | _SLPACK);
}

void mrf24j40_wakeup(int16_t spi_wake) {
  if (spi_wake) {
    mrf24j40_write_short_ctrl_reg(WAKECON, REGWAKE);
    mrf24j40_write_short_ctrl_reg(WAKECON, 0);
  } else {
    mrf24j40_wake_pin(1);
  }

  mrf24j40_rf_reset();
}

void mrf24j40_txpkt(uint8_t *frame, int16_t hdr_len, int16_t sec_hdr_len, int16_t payload_len) {
  int16_t frame_len = hdr_len + sec_hdr_len + payload_len;

  uint8_t w = mrf24j40_read_short_ctrl_reg(TXNCON);
  w &= ~(TXNSECEN);

  if (IEEE_802_15_4_HAS_SEC(frame[0])) {
    w |= TXNSECEN;
  }

  if (IEEE_802_15_4_WANTS_ACK(frame[0])) {
    w |= TXNACKREQ;
  }

  mrf24j40_spi_preamble();
  _mrf24j40_write_long_addr(TXNFIFO, 1);
  mrf24j40_spi_write(hdr_len);
  mrf24j40_spi_write(frame_len);

  while (frame_len-- > 0) {
    mrf24j40_spi_write(*frame++);
  }
  
  mrf24j40_spi_postamble();

  mrf24j40_write_short_ctrl_reg(TXNCON, w | TXNTRIG);
}

void mrf24j40_set_cipher(uint8_t rxcipher, uint8_t txcipher) {
  mrf24j40_write_short_ctrl_reg(SECCON0, RXCIPHER(rxcipher) | TXNCIPHER(txcipher));
}

bool mrf24j40_rx_sec_fail() {
  bool rx_sec_fail = (mrf24j40_read_short_ctrl_reg(RXSR) >> 2) & 0x01;
  mrf24j40_write_short_ctrl_reg(RXSR, 0x00);
  return rx_sec_fail;
}

void mrf24j40_sec_intcb(bool accept) {
  uint8_t w = mrf24j40_read_short_ctrl_reg(SECCON0);

  w |= accept ? SECSTART : SECIGNORE;
  mrf24j40_write_short_ctrl_reg(SECCON0, w);
}

int16_t mrf24j40_txpkt_intcb(void) {
  uint8_t stat = mrf24j40_read_short_ctrl_reg(TXSTAT);
  if (stat & TXNSTAT) {
    if (stat & CCAFAIL) {
      return EBUSY;
    } else {
      return EIO;
    }
  } else {
    return 0;
  }
}

int16_t mrf24j40_rxpkt_intcb(uint8_t *buf, uint8_t *plqi, uint8_t *prssi) {
  mrf24j40_write_short_ctrl_reg(BBREG1, mrf24j40_read_short_ctrl_reg(BBREG1) | RXDECINV);

  mrf24j40_spi_preamble();
  _mrf24j40_write_long_addr(RXFIFO, 0);

  uint16_t flen = mrf24j40_spi_read();

  for (uint16_t i = 0; i < flen; i++) {
    *buf++ = mrf24j40_spi_read();
  }

  uint8_t lqi = mrf24j40_spi_read();
  uint8_t rssi = mrf24j40_spi_read();

  if (plqi != NULL) {
    *plqi = lqi;
  }

  if (prssi != NULL) {
    *prssi = rssi;
  }

  mrf24j40_spi_postamble();

  mrf24j40_rxfifo_flush();
  mrf24j40_write_short_ctrl_reg(BBREG1, mrf24j40_read_short_ctrl_reg(BBREG1) & ~RXDECINV);
  
  return flen;
}

int16_t mrf24j40_int_tasks(void) {
  int16_t ret = 0;

  uint8_t stat = mrf24j40_read_short_ctrl_reg(INTSTAT);

  if (stat & RXIF) {
    ret |= MRF24J40_INT_RX;
  }

  if (stat & TXNIF) {
    ret |= MRF24J40_INT_TX;
  }

  if (stat & SECIF) {
    ret |= MRF24J40_INT_SEC;
  }

  return ret;
}

