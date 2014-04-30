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

#ifndef MRF24J40_H
#define	MRF24J40_H

/* Return values */
#define MRF24J40_INT_RX		0x01
#define MRF24J40_INT_TX		0x02
#define MRF24J40_INT_SEC	0x04
#define MRF24J40_INT_SLP	0x08
#define MRF24J40_INT_ENC	0x10
#define MRF24J40_INT_DEC	0x20

#define EIO			1
#define EBUSY			2

/* IEEE 802.15.4 constants needed for some flags */
#define IEEE_802_15_4_HAS_SEC(x)      ((x >> 3) & 0x01)
#define IEEE_802_15_4_WANTS_ACK(x)     ((x >> 5) & 0x01)

/* Internal state */
#define MRF24J40_STATE_UPENC	0x01
#define MRF24J40_STATE_UPDEC	0x02

/* enc dec parameters */
#define MRF24J40_TX_KEY		0x01
#define MRF24J40_RX_KEY		0x02
#define MRF24J40_UP_KEY		MRF24J40_TX_KEY

#define MRF24J40_AES_CBC_MAC32	0x07
#define MRF24J40_AES_CBC_MAC64	0x06
#define MRF24J40_AES_CBC_MAC128	0x05
#define MRF24J40_AES_CCM32	0x04
#define MRF24J40_AES_CCM64	0x03
#define MRF24J40_AES_CCM128	0x02
#define MRF24J40_AES_CTR	0x01
#define MRF24J40_ALGO_NONE	0x00

/* Short Address Control Register Map */
#define RXMCR		0x00
#define PANIDL		0x01
#define PANIDH		0x02
#define	SADRL		0x03
#define SADRH		0x04
#define EADR0		0x05
#define EADR1		0x06
#define EADR2		0x07
#define EADR3		0x08
#define EADR4		0x09
#define EADR5		0x0A
#define EADR6		0x0B
#define EADR7		0x0C
#define RXFLUSH		0x0D

#define ORDER		0x10
#define TXMCR		0x11
#define ACKTMOUT	0x12
#define ESLOTG1		0x13
#define SYMTICKL	0x14
#define SYMTICKH	0x15
#define PACON0		0x16
#define PACON1		0x17
#define PACON2		0x18
#define TXBCON0		0x1A
#define TXNCON		0x1B
#define TXG1CON		0x1C
#define TXG2CON		0x1D
#define ESLOTG23	0x1E
#define ESLOTG45	0x1F

#define ESLOTG67	0x20
#define TXPEND		0x21
#define WAKECON		0x22
#define FRMOFFSET	0x23
#define TXSTAT		0x24
#define TXBCON1		0x25
#define GATECLK		0x26
#define TXTIME		0x27
#define HSYMTMRL	0x28
#define HSYMTMRH	0x29
#define SOFTRST		0x2A
#define SECCON0		0x2c
#define SECCON1		0x2d
#define TXSTBL		0x2e

#define RXSR		0x30
#define INTSTAT		0x31
#define MRF24J40_INTCON 0x32
#define GPIO		0x33
#define TRISGPIO	0x34
#define SLPACK		0x35
#define RFCTL		0x36
#define SECCR2		0x37
#define BBREG0		0x38
#define BBREG1		0x39
#define BBREG2		0x3A
#define BBREG3		0x3B
#define BBREG4		0x3C
#define BBREG6		0x3E
#define CCAEDTH		0x3F

/* Long Address Control Register Map */
#define RFCON0		0x200
#define RFCON1		0x201
#define RFCON2		0x202
#define RFCON3		0x203
#define RFCON5		0x205
#define RFCON6		0x206
#define RFCON7		0x207
#define RFCON8		0x208
#define RFSTATE		0x20F

#define RSSI		0x210
#define SLPCON0		0x211
#define SLPCON1		0x220

#define WAKETIMEL	0x222
#define WAKETIMEH	0x223

#define MAINCNT0	0x226
#define MAINCNT1	0x227
#define MAINCNT2	0x228
#define MAINCNT3	0x229

#define ASSOEADR0       0x230
#define ASSOEADR1       0x231
#define ASSOEADR2       0x232
#define ASSOEADR3       0x233
#define ASSOEADR4       0x234
#define ASSOEADR5       0x235
#define ASSOEADR6       0x236
#define ASSOEADR7       0x237

#define ASSOSADR0       0x238
#define ASSOSADR1       0x239

#define UPNONCE0	0x240
#define UPNONCE1	0x241
#define UPNONCE2	0x242
#define UPNONCE3	0x243
#define UPNONCE4	0x244
#define UPNONCE5	0x245
#define UPNONCE6	0x246
#define UPNONCE7	0x247
#define UPNONCE8	0x248
#define UPNONCE9	0x249
#define UPNONCE10	0x24A
#define UPNONCE11	0x24B
#define UPNONCE12	0x24C

/* Long Address Memory Map */
#define TXNFIFO		0x000 /* - 0x07F, 128 bytes */
#define TXBFIFO		0x080 /* - 0x0FF, 128 bytes */
#define TXG1FIFO	0x100 /* - 0x17F, 128 bytes */
#define TXG2FIFO	0x180 /* - 0x1FF, 128 bytes */
#define SECKFIFO	0x280 /* - 0x2BF, 64 bytes */
#define SECKTXNFIFO	0x280 /* - 0x28F, 16 bytes */
#define SECKRXFIFO	0x2B0 /* - 0x2BF, 16 bytes */
#define RXFIFO		0x300 /* - 0x38F, 128 bytes */


/* RXMCR */
#define NOACKRSP	(1<<5)
#define PANCOORD	(1<<3)
#define COORD		(1<<2)
#define ERRPKT		(1<<1)
#define PROMI		(1)

/* RXFLUSH */
#define WAKEPOL		(1<<6)
#define WAKEPAD		(1<<5)
#define CMDONLY		(1<<3)
#define DATAONLY	(1<<2)
#define BCNONLY		(1<<1)
#define _RXFLUSH	(1)

/* TXMCR */
#define NOCSMA		(1<<7)
#define BATLIFEXT	(1<<6)
#define SLOTTED		(1<<5)
#define MACMINBE(x)	((x & 0x03)<<3)
#define CSMABF(x)	(x & 0x07)

/* ACKTMOUT */
#define DRPACK		(1<<7)

/* PACON2 */
#define FIFOEN		(1<<7)
#define TXONTS(x)       (x & 0x3F)

/* TXNCON */
#define FPSTAT		(1<<4)
#define INDIRECT	(1<<3)
#define TXNACKREQ	(1<<2)
#define TXNSECEN	(1<<1)
#define TXNTRIG		(1)

/* TXPEND */
#define FPACK		(1)

/* WAKECON */
#define IMMWAKE		(1<<7)
#define REGWAKE		(1<<6)

/* TXSTAT */
#define CCAFAIL		(1<<5)
#define TXNSTAT		(1)

/* SOFTRST */
#define RSTPWR		(1<<2)
#define RSTBB		(1<<1)
#define RSTMAC		(1)

/* SECCON0 */
#define SECIGNORE	(1<<7)
#define SECSTART	(1<<6)
#define RXCIPHER(x)	((x & 0x07) << 3)
#define TXNCIPHER(x)	((x & 0x07))

/* SECCON1 */
#define DISDEC		(1<<1)
#define DISENC		(1)

/* TXSTBL */
#define RFSTBL(x)	((x & 0x0f) << 4)
#define MSIFS(x)	((x & 0x0f))

/* RXSR */
#define UPSECERR	(1<<6)
#define SECDECERR	(1<<2)


/* INTSTAT */
#define SLPIF		(1<<7)
#define WAKEIF		(1<<6)
#define HSYMTMRIF	(1<<5)
#define SECIF		(1<<4)
#define RXIF		(1<<3)
#define TXG2IF		(1<<2)
#define TXG1IF		(1<<1)
#define TXNIF		(1)

/* INTCON */
#define SLPIE		(1<<7)
#define WAKEIE		(1<<6)
#define HSYMTMRIE	(1<<5)
#define SECIE		(1<<4)
#define RXIE		(1<<3)
#define TXG2IE		(1<<2)
#define TXG1IE		(1<<1)
#define TXNIE		(1)

/* SLPACK */
#define _SLPACK		(1<<7)
#define WAKECNT_L(x)	(x & 0x03F)

/* RFCTL */
#define WAKECNT_H(x)	((x & 0x03) << 3)
#define RFRST		(1<<2)
#define RFTXMODE	(1<<1)
#define RFRXMODE	(1)

/* SECCR2 */
#define UPDEC		(1<<7)
#define UPENC		(1<<6)

/* BBREG0 */
#define TURBO		(1)

/* BBREG1 */
#define RXDECINV	(1<<2)

/* BBREG2 */
#define CCAMODE(x)	((x & 0x03) <<6)
#define CCASTH(x)	((x & 0x0F) <<2)

/* BBREG3 */
#define PREVALIDTH(x)	((x & 0x0F) <<4)

/* BBREG4 */
#define CSTH(x)		((x & 0x07) << 5)

/* BBREG6 */
#define RSSIMODE1	(1 << 7)
#define RSSIMODE2	(1<<6)
#define RSSIRDY		(1)

/* RFCON0 */
#define CHANNEL(x)	((x & 0x0F) << 4)
#define RFOPT(x)	((x & 0x0F))

/* RFCON1 */
#define VCOOPT(x)	((x & 0xFF))

/* RFCON2 */
#define PLLEN		(1<<7)

/* RFCON3 */
#define TXPWRL(x)	((x & 0x03) << 6)
#define TXPWRS(x)	((x & 0x07) << 3)

/* RFCON6 */
#define TXFIL		(1 << 7)
#define _20MRECVR       (1 << 4)
#define BATEN           (1 << 3)

/* RFCON 7 */
#define SLPCLKSEL(x)	((x & 0x03) << 6)
#define SLPCLKSEL_100k	(SLPCLKSEL(0x02))
#define SLPCLKSEL_32k	(SLPCLKSEL(0x01))

/* RFCON8 */
#define RFVCO		(1 << 4)

/* SLPCON0 */
#define SLPCLKEN	(1)

/* SLPCON1 */
#define CLKOUTDIS	(1 << 5)	/* CLKOUTEN' */
#define SLPCLKDIV(x)	((x & 0x1F))	/* division ratio: 2^(SLPCLKDIV) */


unsigned char mrf24j40_read_long_ctrl_reg(unsigned short addr);
unsigned char mrf24j40_read_short_ctrl_reg(unsigned char addr);
void mrf24j40_write_long_ctrl_reg(unsigned short addr, unsigned char value);
void mrf24j40_write_short_ctrl_reg(unsigned char addr, unsigned char value);
void mrf24j40_rxfifo_flush(void);
void mrf24j40_hard_reset(void);
void mrf24j40_initialize(void);
void mrf24j40_sleep(int spi_wake);
void mrf24j40_wakeup(int spi_wake);
void mrf24j40_set_short_addr(unsigned char *addr);
void mrf24j40_set_coordinator_short_addr(unsigned char *addr);
void mrf24j40_set_coordinator_eui(unsigned char *eui);
void mrf24j40_set_eui(unsigned char *eui);
void mrf24j40_set_pan(unsigned char *pan);
void mrf24j40_set_channel(int ch);
void mrf24j40_set_promiscuous(int crc_check);
void mrf24j40_set_coordinator(void);
void mrf24j40_clear_coordinator(void);
void mrf24j40_txpkt(unsigned char *frame, int hdr_len, int payload_len);
unsigned char mrf24j40_get_channel(void);
int mrf24j40_int_tasks(void);
int mrf24j40_rxpkt_intcb(unsigned char *buf, unsigned char *plqi, unsigned char *prssi);
int mrf24j40_txpkt_intcb(void);

#ifdef MRF24J40_UPPER_LAYER_ENC_DEC
int mrf24j40_sec_intcb(int accept);
int mrf24j40_check_rx_dec(int no_err_flush);
int mrf24j40_check_enc(void);
int mrf24j40_check_dec(void);
void mrf24j40_set_encdec(int types, int mode, unsigned char *key, int klen);
void mrf24j40_encdec(unsigned char *nonce, int nonce_len, unsigned char *frame, int hdr_len, int payload_len, int enc);
#endif

#endif /* MRF24J40_H */