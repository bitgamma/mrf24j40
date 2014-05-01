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

#include <xc.h>
#include <stdlib.h>

static int internal_state = 0;

#define mrf24j40_spi_preamble() volatile unsigned char tmpIE = MRF24J40_IE; MRF24J40_IE = 0; MRF24J40_CS = 0;
#define mrf24j40_spi_postamble() MRF24J40_CS = 1; MRF24J40_IE = tmpIE;

void _mrf24j40_write_long_addr(unsigned short addr, unsigned char write) {
  mrf24j40_spi_write(((addr >> 3) & 0x7F) | 0x80);
  mrf24j40_spi_write(((addr << 5) & 0xE0) | (write << 4));
}

void _mrf24j40_write_short_addr(unsigned char addr, unsigned char write) {
  mrf24j40_spi_write(((addr << 1) & 0x7E) | write);
}

unsigned char mrf24j40_read_long_ctrl_reg(unsigned short addr) {
  mrf24j40_spi_preamble();
  _mrf24j40_write_long_addr(addr, 0);
  unsigned char value = mrf24j40_spi_read();
  mrf24j40_spi_postamble();

  return value;
}

unsigned char mrf24j40_read_short_ctrl_reg(unsigned char addr) {
  mrf24j40_spi_preamble();
  _mrf24j40_write_short_addr(addr, 0);
  unsigned char value = mrf24j40_spi_read();
  mrf24j40_spi_postamble();
  return value;
}

void mrf24j40_write_long_ctrl_reg(unsigned short addr, unsigned char value) {
  mrf24j40_spi_preamble();
  _mrf24j40_write_long_addr(addr, 1);
  mrf24j40_spi_write(value);
  mrf24j40_spi_postamble();
}

void mrf24j40_write_short_ctrl_reg(unsigned char addr, unsigned char value) {
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
  internal_state = 0;
  mrf24j40_write_short_ctrl_reg(SOFTRST, RSTMAC);
}

void mrf24j40_rf_reset(void) {
  unsigned char old = mrf24j40_read_short_ctrl_reg(RFCTL);

  mrf24j40_write_short_ctrl_reg(RFCTL, old | RFRST);
  mrf24j40_write_short_ctrl_reg(RFCTL, old & ~RFRST);
  __delay_ms(2);
}

unsigned char mrf24j40_get_pending_frame(void) {
  return (mrf24j40_read_short_ctrl_reg(TXNCON) >> 4) & 0x01;
}

void mrf24j40_rxfifo_flush(void) {
  mrf24j40_write_short_ctrl_reg(RXFLUSH, (mrf24j40_read_short_ctrl_reg(RXFLUSH) | _RXFLUSH));
}

void mrf24j40_set_channel(int ch) {
  mrf24j40_write_long_ctrl_reg(RFCON0, CHANNEL(ch) | RFOPT(0x03));
  mrf24j40_rf_reset();
}

void mrf24j40_set_promiscuous(int crc_check) {
  unsigned char w = NOACKRSP;
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

void mrf24j40_set_pan(unsigned char *pan) {
  mrf24j40_write_short_ctrl_reg(PANIDL, pan[0]);
  mrf24j40_write_short_ctrl_reg(PANIDH, pan[1]);
}

void mrf24j40_set_short_addr(unsigned char *addr) {
  mrf24j40_write_short_ctrl_reg(SADRL, addr[0]);
  mrf24j40_write_short_ctrl_reg(SADRH, addr[1]);
}

void mrf24j40_set_eui(unsigned char *eui) {
  mrf24j40_write_short_ctrl_reg(EADR0, eui[0]);
  mrf24j40_write_short_ctrl_reg(EADR1, eui[1]);
  mrf24j40_write_short_ctrl_reg(EADR2, eui[2]);
  mrf24j40_write_short_ctrl_reg(EADR3, eui[3]);
  mrf24j40_write_short_ctrl_reg(EADR4, eui[4]);
  mrf24j40_write_short_ctrl_reg(EADR5, eui[5]);
  mrf24j40_write_short_ctrl_reg(EADR6, eui[6]);
  mrf24j40_write_short_ctrl_reg(EADR7, eui[7]);
}

void mrf24j40_set_coordinator_short_addr(unsigned char *addr) {
  mrf24j40_write_short_ctrl_reg(ASSOSADR0, addr[0]);
  mrf24j40_write_short_ctrl_reg(ASSOSADR1, addr[1]);
}

void mrf24j40_set_coordinator_eui(unsigned char *eui) {
  mrf24j40_write_short_ctrl_reg(ASSOEADR0, eui[0]);
  mrf24j40_write_short_ctrl_reg(ASSOEADR1, eui[1]);
  mrf24j40_write_short_ctrl_reg(ASSOEADR2, eui[2]);
  mrf24j40_write_short_ctrl_reg(ASSOEADR3, eui[3]);
  mrf24j40_write_short_ctrl_reg(ASSOEADR4, eui[4]);
  mrf24j40_write_short_ctrl_reg(ASSOEADR5, eui[5]);
  mrf24j40_write_short_ctrl_reg(ASSOEADR6, eui[6]);
  mrf24j40_write_short_ctrl_reg(ASSOEADR7, eui[7]);
}

void mrf24j40_hard_reset() {
  MRF24J40_RESET = 0;

  internal_state = 0;
  __delay_us(192);

  MRF24J40_RESET = 1;

  __delay_us(192);
}

void mrf24j40_initialize() {
  MRF24J40_CS = 1;
  MRF24J40_WAKE = 0;
  
  mrf24j40_hard_reset();
  
  mrf24j40_write_short_ctrl_reg(SOFTRST, (RSTPWR | RSTBB | RSTMAC));

  __delay_us(192);

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

void mrf24j40_sleep(int spi_wake) {
  mrf24j40_write_short_ctrl_reg(WAKECON, IMMWAKE);

  unsigned char r = mrf24j40_read_short_ctrl_reg(SLPACK);

  if (!spi_wake) {
    MRF24J40_WAKE = 0;
    mrf24j40_write_short_ctrl_reg(RXFLUSH, mrf24j40_read_short_ctrl_reg(RXFLUSH) | WAKEPAD | WAKEPOL);
  }

  mrf24j40_pwr_reset();
  mrf24j40_write_short_ctrl_reg(SLPACK, r | _SLPACK);
}

void mrf24j40_wakeup(int spi_wake) {
  if (spi_wake) {
    mrf24j40_write_short_ctrl_reg(WAKECON, REGWAKE);
    mrf24j40_write_short_ctrl_reg(WAKECON, 0);
  } else {
    MRF24J40_WAKE = 1;
  }

  mrf24j40_rf_reset();
}

void mrf24j40_txpkt(unsigned char *frame, int hdr_len, int payload_len) {
  internal_state = 0;
  int frame_len = hdr_len+payload_len;

  unsigned char w = mrf24j40_read_short_ctrl_reg(TXNCON);
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

int mrf24j40_txpkt_intcb(void) {
  unsigned char stat = mrf24j40_read_short_ctrl_reg(TXSTAT);
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

int mrf24j40_rxpkt_intcb(unsigned char *buf, unsigned char *plqi, unsigned char *prssi) {
  mrf24j40_write_short_ctrl_reg(BBREG1, mrf24j40_read_short_ctrl_reg(BBREG1) | RXDECINV);

  mrf24j40_spi_preamble();
  _mrf24j40_write_long_addr(RXFIFO, 0);

  unsigned int flen = mrf24j40_spi_read();

  for (unsigned int i = 0; i < flen; i++) {
    *buf++ = mrf24j40_spi_read();
  }

  unsigned char lqi = mrf24j40_spi_read();
  unsigned char rssi = mrf24j40_spi_read();

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

int mrf24j40_int_tasks(void) {
  int ret = 0;

  unsigned char stat = mrf24j40_read_short_ctrl_reg(INTSTAT);

  if (stat & RXIF) {
    ret |= MRF24J40_INT_RX;
  }

  if (stat & TXNIF) {
    switch (internal_state) {
      case MRF24J40_STATE_UPENC:
        ret |= MRF24J40_INT_ENC;
        internal_state = 0;
        break;

      case MRF24J40_STATE_UPDEC:
        ret |= MRF24J40_INT_DEC;
        internal_state = 0;
        break;

      default:
        ret |= MRF24J40_INT_TX;
    }
  }

  if (stat & SECIF) {
    ret |= MRF24J40_INT_SEC;
  }

  return ret;
}

#ifdef MRF24J40_UPPER_LAYER_ENC_DEC
int mrf24j40_sec_intcb(int accept) {
  unsigned char w = mrf24j40_read_short_ctrl_reg(SECCON0);
  
  w &= ~(SECSTART | SECIGNORE);

  if (accept) {
    mrf24j40_write_short_ctrl_reg(SECCON0, w | SECSTART);
  } else {
    mrf24j40_write_short_ctrl_reg(SECCON0, w | SECIGNORE);
    mrf24j40_write_short_ctrl_reg(RXFLUSH, _RXFLUSH);
  }
}

int mrf24j40_check_rx_dec(int no_err_flush) {
  int err = (mrf24j40_read_short_ctrl_reg(RXSR) & SECDECERR) ? EIO : 0;

  if (err && !no_err_flush) {
    mrf24j40_write_short_ctrl_reg(RXFLUSH, _RXFLUSH);
  }

  return err;
}

int mrf24j40_check_enc(void) {
  return mrf24j40_txpkt_intcb();
}

int mrf24j40_check_dec(void) {
  int error;

  if ((error = mrf24j40_txpkt_intcb()) != 0) {
    return error;
  }

  unsigned char w = mrf24j40_read_short_ctrl_reg(RXSR);
  return (w & UPSECERR) ? EIO : 0;
}

void mrf24j40_set_encdec(int types, int mode, unsigned char *key, int klen) {
  unsigned char *keyp;

  unsigned char w = mrf24j40_read_short_ctrl_reg(SECCON0);

  if (types & MRF24J40_TX_KEY) {
    mrf24j40_spi_preamble();
    _mrf24j40_write_long_addr(SECKTXNFIFO, 1);

    keyp = key;

    int len = klen;

    while (len-- > 0) {
      mrf24j40_spi_write(*keyp++);
    }

    mrf24j40_spi_postamble();


    w |= TXNCIPHER(mode);
    mrf24j40_write_short_ctrl_reg(SECCON0, w);
  }

  if (types & MRF24J40_RX_KEY) {
    mrf24j40_spi_preamble();
    _mrf24j40_write_long_addr(SECKRXFIFO, 1);

    keyp = key;
    int len = klen;

    while (len-- > 0) {
      mrf24j40_spi_write(*keyp++);
    }

    mrf24j40_spi_postamble();

    w |= RXCIPHER(mode);
    mrf24j40_write_short_ctrl_reg(SECCON0, w);
  }
}

void mrf24j40_encdec(unsigned char *nonce, int nonce_len, unsigned char *frame, int hdr_len, int payload_len, int enc) {
  if (enc) {
    internal_state = MRF24J40_STATE_UPENC;
  } else {
    internal_state = MRF24J40_STATE_UPDEC;
  }

  int addr = UPNONCE0;
  while (nonce_len-- > 0) {
    mrf24j40_write_long_ctrl_reg(addr++, *nonce++);
  }

  if (enc) {
    mrf24j40_write_short_ctrl_reg(SECCR2, UPENC);
  } else {
    mrf24j40_write_short_ctrl_reg(SECCR2, UPDEC);
  }

  mrf24j40_txpkt(frame, hdr_len, payload_len);
}
#endif
