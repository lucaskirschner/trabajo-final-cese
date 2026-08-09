/******************************************************************************
 * Copyright (c) 2026, Lucas Kirschner <kirschnerlucas1@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * SPDX-License-Identifier: MIT
 ******************************************************************************/

/**
 * @file    mrf24j40_reg.h
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-02-12
 * @brief   MRF24J40 control register map and bitfield definitions.
 *
 * @details
 * Register addresses and bitfield macros based on datasheet control register
 * summaries and register detail sections (Tables 2-6 and 2-7, and Register 2-x
 * descriptions).
 *
 * Naming:
 * - Register names and bit/field names match the datasheet exactly.
 * - Short address registers use 6-bit address space (0x00..0x3F).
 * - Long address registers use 10-bit address space (0x200..).
 *
 * @ingroup mrf24j40
 * @{
 */

#ifndef MRF24J40_REG_H_
#define MRF24J40_REG_H_

/* ============================= Includes ================================== */

#include <stdint.h>

/* ============================== Macros =================================== */

/* MEMORY MAP FOR MRF24J40 */
#define TX_NORMAL_FIFO                         ((uint16_t)0x000u)
#define TX_BEACON_FIFO                         ((uint16_t)0x080u)
#define TX_GTS1_FIFO                           ((uint16_t)0x100u)
#define TX_GTS2_FIFO                           ((uint16_t)0x180u)
#define RX_FIFO                                ((uint16_t)0x300u)
#define SEC_KEY                                ((uint16_t)0x280u)

/* SHORT ADDRESS CONTROL REGISTER MAP FOR MRF24J40 */
#define RXMCR                                  ((uint8_t)0x00u)
#define PANIDL                                 ((uint8_t)0x01u)
#define PANIDH                                 ((uint8_t)0x02u)
#define SADRL                                  ((uint8_t)0x03u)
#define SADRH                                  ((uint8_t)0x04u)
#define EADR0                                  ((uint8_t)0x05u)
#define EADR1                                  ((uint8_t)0x06u)
#define EADR2                                  ((uint8_t)0x07u)
#define EADR3                                  ((uint8_t)0x08u)
#define EADR4                                  ((uint8_t)0x09u)
#define EADR5                                  ((uint8_t)0x0Au)
#define EADR6                                  ((uint8_t)0x0Bu)
#define EADR7                                  ((uint8_t)0x0Cu)
#define RXFLUSH                                ((uint8_t)0x0Du)
#define ORDER                                  ((uint8_t)0x10u)
#define TXMCR                                  ((uint8_t)0x11u)
#define ACKTMOUT                               ((uint8_t)0x12u)
#define ESLOTG1                                ((uint8_t)0x13u)
#define SYMTICKL                               ((uint8_t)0x14u)
#define SYMTICKH                               ((uint8_t)0x15u)
#define PACON0                                 ((uint8_t)0x16u)
#define PACON1                                 ((uint8_t)0x17u)
#define PACON2                                 ((uint8_t)0x18u)
#define TXBCON0                                ((uint8_t)0x1Au)
#define TXNCON                                 ((uint8_t)0x1Bu)
#define TXG1CON                                ((uint8_t)0x1Cu)
#define TXG2CON                                ((uint8_t)0x1Du)
#define ESLOTG23                               ((uint8_t)0x1Eu)
#define ESLOTG45                               ((uint8_t)0x1Fu)
#define ESLOTG67                               ((uint8_t)0x20u)
#define TXPEND                                 ((uint8_t)0x21u)
#define WAKECON                                ((uint8_t)0x22u)
#define FRMOFFSET                              ((uint8_t)0x23u)
#define TXSTAT                                 ((uint8_t)0x24u)
#define TXBCON1                                ((uint8_t)0x25u)
#define GATECLK                                ((uint8_t)0x26u)
#define TXTIME                                 ((uint8_t)0x27u)
#define HSYMTMRL                               ((uint8_t)0x28u)
#define HSYMTMRH                               ((uint8_t)0x29u)
#define SOFTRST                                ((uint8_t)0x2Au)
#define SECCON0                                ((uint8_t)0x2Cu)
#define SECCON1                                ((uint8_t)0x2Du)
#define TXSTBL                                 ((uint8_t)0x2Eu)
#define RXSR                                   ((uint8_t)0x30u)
#define INTSTAT                                ((uint8_t)0x31u)
#define INTCON                                 ((uint8_t)0x32u)
#define GPIO                                   ((uint8_t)0x33u)
#define TRISGPIO                               ((uint8_t)0x34u)
#define SLPACK                                 ((uint8_t)0x35u)
#define RFCTL                                  ((uint8_t)0x36u)
#define SECCR2                                 ((uint8_t)0x37u)
#define BBREG0                                 ((uint8_t)0x38u)
#define BBREG1                                 ((uint8_t)0x39u)
#define BBREG2                                 ((uint8_t)0x3Au)
#define BBREG3                                 ((uint8_t)0x3Bu)
#define BBREG4                                 ((uint8_t)0x3Cu)
#define BBREG6                                 ((uint8_t)0x3Eu)
#define CCAEDTH                                ((uint8_t)0x3Fu)

/* LONG ADDRESS CONTROL REGISTER MAP FOR MRF24J40 */
#define RFCON0                                 ((uint16_t)0x0200u)
#define RFCON1                                 ((uint16_t)0x0201u)
#define RFCON2                                 ((uint16_t)0x0202u)
#define RFCON3                                 ((uint16_t)0x0203u)
#define RFCON5                                 ((uint16_t)0x0205u)
#define RFCON6                                 ((uint16_t)0x0206u)
#define RFCON7                                 ((uint16_t)0x0207u)
#define RFCON8                                 ((uint16_t)0x0208u)
#define SLPCAL0                                ((uint16_t)0x0209u)
#define SLPCAL1                                ((uint16_t)0x020Au)
#define SLPCAL2                                ((uint16_t)0x020Bu)
#define RFSTATE                                ((uint16_t)0x020Fu)
#define RSSI                                   ((uint16_t)0x0210u)
#define SLPCON0                                ((uint16_t)0x0211u)
#define SLPCON1                                ((uint16_t)0x0220u)
#define WAKETIMEL                              ((uint16_t)0x0222u)
#define WAKETIMEH                              ((uint16_t)0x0223u)
#define REMCNTL                                ((uint16_t)0x0224u)
#define REMCNTH                                ((uint16_t)0x0225u)
#define MAINCNT0                               ((uint16_t)0x0226u)
#define MAINCNT1                               ((uint16_t)0x0227u)
#define MAINCNT2                               ((uint16_t)0x0228u)
#define MAINCNT3                               ((uint16_t)0x0229u)
#define TESTMODE                               ((uint16_t)0x022Fu)
#define ASSOEADR0                              ((uint16_t)0x0230u)
#define ASSOEADR1                              ((uint16_t)0x0231u)
#define ASSOEADR2                              ((uint16_t)0x0232u)
#define ASSOEADR3                              ((uint16_t)0x0233u)
#define ASSOEADR4                              ((uint16_t)0x0234u)
#define ASSOEADR5                              ((uint16_t)0x0235u)
#define ASSOEADR6                              ((uint16_t)0x0236u)
#define ASSOEADR7                              ((uint16_t)0x0237u)
#define ASSOSADR0                              ((uint16_t)0x0238u)
#define ASSOSADR1                              ((uint16_t)0x0239u)

/* SHORT ADDRESS CONTROL REGISTER SUMMARY FOR MRF24J40 */
/* ===================== RXMCR Register Definitions ======================== */
#define RXMCR_NOACKRSP                         ((uint8_t)0x20u)
#define RXMCR_PANCOORD                         ((uint8_t)0x08u)
#define RXMCR_COORD                            ((uint8_t)0x04u)
#define RXMCR_ERRPKT                           ((uint8_t)0x02u)
#define RXMCR_PROMI                            ((uint8_t)0x01u)

/* ===================== RXFLUSH Register Definitions ======================= */
#define RXFLUSH_WAKEPOL                        ((uint8_t)0x40u)
#define RXFLUSH_WAKEPAD                        ((uint8_t)0x20u)
#define RXFLUSH_CMDONLY                        ((uint8_t)0x08u)
#define RXFLUSH_DATAONLY                       ((uint8_t)0x04u)
#define RXFLUSH_BCNONLY                        ((uint8_t)0x02u)
#define RXFLUSH_RXFLUSH                        ((uint8_t)0x01u)

/* ====================== TXMCR Register Definitions ======================== */
#define TXMCR_NOCSMA                           ((uint8_t)0x80u)
#define TXMCR_BATLIFEXT                        ((uint8_t)0x40u)
#define TXMCR_SLOTTED                          ((uint8_t)0x20u)
#define TXMCR_MACMINBE1                        ((uint8_t)0x10u)
#define TXMCR_MACMINBE0                        ((uint8_t)0x08u)
#define TXMCR_CSMABF2                          ((uint8_t)0x04u)
#define TXMCR_CSMABF1                          ((uint8_t)0x02u)
#define TXMCR_CSMABF0                          ((uint8_t)0x01u)

/* ===================== ACKTMOUT Register Definitions ====================== */
#define ACKTMOUT_DRPACK                        ((uint8_t)0x80u)
#define ACKTMOUT_MAWD6                         ((uint8_t)0x40u)
#define ACKTMOUT_MAWD5                         ((uint8_t)0x20u)
#define ACKTMOUT_MAWD4                         ((uint8_t)0x10u)
#define ACKTMOUT_MAWD3                         ((uint8_t)0x08u)
#define ACKTMOUT_MAWD2                         ((uint8_t)0x04u)
#define ACKTMOUT_MAWD1                         ((uint8_t)0x02u)
#define ACKTMOUT_MAWD0                         ((uint8_t)0x01u)

/* ===================== TXBCON0 Register Definitions ======================= */
#define TXBCON0_TXBSECEN                       ((uint8_t)0x02u)
#define TXBCON0_TXBTRIG                        ((uint8_t)0x01u)

/* ====================== TXNCON Register Definitions ======================= */
#define TXNCON_FPSTAT                          ((uint8_t)0x10u)
#define TXNCON_INDIRECT                        ((uint8_t)0x08u)
#define TXNCON_TXNACKREQ                       ((uint8_t)0x04u)
#define TXNCON_TXNSECEN                        ((uint8_t)0x02u)
#define TXNCON_TXNTRIG                         ((uint8_t)0x01u)

/* ===================== WAKECON Register Definitions ======================= */
#define WAKECON_IMMWAKE                        ((uint8_t)0x80u)
#define WAKECON_REGWAKE                        ((uint8_t)0x40u)

/* ===================== TXBCON1 Register Definitions ======================= */
#define TXBCON1_TXBMSK                         ((uint8_t)0x80u)
#define TXBCON1_WU_BCN                         ((uint8_t)0x40u)
#define TXBCON1_RSSINUM1                       ((uint8_t)0x20u)
#define TXBCON1_RSSINUM0                       ((uint8_t)0x10u)

/* RSSINUM[1:0] preset values (bits [5:4]) */
#define TXBCON1_RSSINUM_8_SYMBOL               ((uint8_t)0x30u)
#define TXBCON1_RSSINUM_4_SYMBOL               ((uint8_t)0x20u)
#define TXBCON1_RSSINUM_2_SYMBOL               ((uint8_t)0x10u)
#define TXBCON1_RSSINUM_1_SYMBOL               ((uint8_t)0x00u)

/* ===================== GATECLK Register Definitions ======================= */
#define GATECLK_GTSON                          ((uint8_t)0x08u)

/* ===================== SOFTRST Register Definitions ======================= */
#define SOFTRST_RSTPWR                         ((uint8_t)0x04u)
#define SOFTRST_RSTBB                          ((uint8_t)0x02u)
#define SOFTRST_RSTMAC                         ((uint8_t)0x01u)

/* ===================== SECCON0 Register Definitions ======================= */
#define SECCON0_SECIGNORE                      ((uint8_t)0x80u)
#define SECCON0_SECSTART                       ((uint8_t)0x40u)

/* RXCIPHER[2:0] (bits [5:3]) */
#define SECCON0_RXCIPHER2                      ((uint8_t)0x20u)
#define SECCON0_RXCIPHER1                      ((uint8_t)0x10u)
#define SECCON0_RXCIPHER0                      ((uint8_t)0x08u)

/* TXNCIPHER[2:0] (bits [2:0]) */
#define SECCON0_TXNCIPHER2                     ((uint8_t)0x04u)
#define SECCON0_TXNCIPHER1                     ((uint8_t)0x02u)
#define SECCON0_TXNCIPHER0                     ((uint8_t)0x01u)

/* RXCIPHER preset values (bits [5:3]) */
#define SECCON0_RXCIPHER_NONE                  ((uint8_t)0x00u)
#define SECCON0_RXCIPHER_AES_CTR               ((uint8_t)0x08u)
#define SECCON0_RXCIPHER_AES_CCM_128           ((uint8_t)0x10u)
#define SECCON0_RXCIPHER_AES_CCM_64            ((uint8_t)0x18u)
#define SECCON0_RXCIPHER_AES_CCM_32            ((uint8_t)0x20u)
#define SECCON0_RXCIPHER_AES_CBC_MAC_128       ((uint8_t)0x28u)
#define SECCON0_RXCIPHER_AES_CBC_MAC_64        ((uint8_t)0x30u)
#define SECCON0_RXCIPHER_AES_CBC_MAC_32        ((uint8_t)0x38u)

/* TXNCIPHER preset values (bits [2:0]) */
#define SECCON0_TXNCIPHER_NONE                 ((uint8_t)0x00u)
#define SECCON0_TXNCIPHER_AES_CTR              ((uint8_t)0x01u)
#define SECCON0_TXNCIPHER_AES_CCM_128          ((uint8_t)0x02u)
#define SECCON0_TXNCIPHER_AES_CCM_64           ((uint8_t)0x03u)
#define SECCON0_TXNCIPHER_AES_CCM_32           ((uint8_t)0x04u)
#define SECCON0_TXNCIPHER_AES_CBC_MAC_128      ((uint8_t)0x05u)
#define SECCON0_TXNCIPHER_AES_CBC_MAC_64       ((uint8_t)0x06u)
#define SECCON0_TXNCIPHER_AES_CBC_MAC_32       ((uint8_t)0x07u)

/* ===================== SECCON1 Register Definitions ======================= */
/* TXBCIPHER[2:0] (bits [6:4]) */
#define SECCON1_TXBCIPHER2                     ((uint8_t)0x40u)
#define SECCON1_TXBCIPHER1                     ((uint8_t)0x20u)
#define SECCON1_TXBCIPHER0                     ((uint8_t)0x10u)

#define SECCON1_DISDEC                         ((uint8_t)0x02u)
#define SECCON1_DISENC                         ((uint8_t)0x01u)

/* TXBCIPHER preset values (bits [6:4]) */
#define SECCON1_TXBCIPHER_NONE                 ((uint8_t)0x00u)
#define SECCON1_TXBCIPHER_AES_CTR              ((uint8_t)0x10u)
#define SECCON1_TXBCIPHER_AES_CCM_128          ((uint8_t)0x20u)
#define SECCON1_TXBCIPHER_AES_CCM_64           ((uint8_t)0x30u)
#define SECCON1_TXBCIPHER_AES_CCM_32           ((uint8_t)0x40u)
#define SECCON1_TXBCIPHER_AES_CBC_MAC_128      ((uint8_t)0x50u)
#define SECCON1_TXBCIPHER_AES_CBC_MAC_64       ((uint8_t)0x60u)
#define SECCON1_TXBCIPHER_AES_CBC_MAC_32       ((uint8_t)0x70u)

/* ====================== PACON2 Register Definitions ======================= */
#define PACON2_FIFOEN                          ((uint8_t)0x80u)
#define PACON2_TXONTS3                         ((uint8_t)0x20u)
#define PACON2_TXONTS2                         ((uint8_t)0x10u)
#define PACON2_TXONTS1                         ((uint8_t)0x08u)
#define PACON2_TXONTS0                         ((uint8_t)0x04u)
#define PACON2_TXONT8                          ((uint8_t)0x02u)
#define PACON2_TXONT7                          ((uint8_t)0x01u)

/* ====================== TXSTBL Register Definitions ======================= */
#define TXSTBL_RFSTBL3                         ((uint8_t)0x80u)
#define TXSTBL_RFSTBL2                         ((uint8_t)0x40u)
#define TXSTBL_RFSTBL1                         ((uint8_t)0x20u)
#define TXSTBL_RFSTBL0                         ((uint8_t)0x10u)
#define TXSTBL_MSIFS3                          ((uint8_t)0x08u)
#define TXSTBL_MSIFS2                          ((uint8_t)0x04u)
#define TXSTBL_MSIFS1                          ((uint8_t)0x02u)
#define TXSTBL_MSIFS0                          ((uint8_t)0x01u)

/* ======================= RXSR Register Definitions ======================== */
#define RXSR_UPSECERR                          ((uint8_t)0x40u)
#define RXSR_BATIND                            ((uint8_t)0x20u)
#define RXSR_SECDECERR                         ((uint8_t)0x04u)

/* ===================== INTSTAT Register Definitions ======================= */
#define INTSTAT_SLPIF                          ((uint8_t)0x80u)
#define INTSTAT_WAKEIF                         ((uint8_t)0x40u)
#define INTSTAT_HSYMTMRIF                      ((uint8_t)0x20u)
#define INTSTAT_SECIF                          ((uint8_t)0x10u)
#define INTSTAT_RXIF                           ((uint8_t)0x08u)
#define INTSTAT_TXG2IF                         ((uint8_t)0x04u)
#define INTSTAT_TXG1IF                         ((uint8_t)0x02u)
#define INTSTAT_TXNIF                          ((uint8_t)0x01u)

/* ====================== INTCON Register Definitions ======================= */
/* NOTE: In INTCON, a '1' disables the interrupt. Keep names per datasheet. */
#define INTCON_SLPIE                           ((uint8_t)0x80u)
#define INTCON_WAKEIE                          ((uint8_t)0x40u)
#define INTCON_HSYMTMRIE                       ((uint8_t)0x20u)
#define INTCON_SECIE                           ((uint8_t)0x10u)
#define INTCON_RXIE                            ((uint8_t)0x08u)
#define INTCON_TXG2IE                          ((uint8_t)0x04u)
#define INTCON_TXG1IE                          ((uint8_t)0x02u)
#define INTCON_TXNIE                           ((uint8_t)0x01u)

/* ====================== RFCTL Register Definitions ======================== */
#define RFCTL_WAKECNT8                         ((uint8_t)0x10u)
#define RFCTL_WAKECNT7                         ((uint8_t)0x08u)
#define RFCTL_RFRST                            ((uint8_t)0x04u)
#define RFCTL_RFTXMODE                         ((uint8_t)0x02u)
#define RFCTL_RFRXMODE                         ((uint8_t)0x01u)

/* ====================== BBREG0 Register Definitions ======================= */
#define BBREG0_TURBO                           ((uint8_t)0x01u)

/* ====================== BBREG1 Register Definitions ======================= */
#define BBREG1_RXDECINV                        ((uint8_t)0x04u)

/* ====================== BBREG2 Register Definitions ======================= */
#define BBREG2_CCAMODE1                        ((uint8_t)0x80u)
#define BBREG2_CCAMODE0                        ((uint8_t)0x40u)
#define BBREG2_CCACSTH3                        ((uint8_t)0x20u)
#define BBREG2_CCACSTH2                        ((uint8_t)0x10u)
#define BBREG2_CCACSTH1                        ((uint8_t)0x08u)
#define BBREG2_CCACSTH0                        ((uint8_t)0x04u)

/* CCAMODE[1:0] preset values (bits [7:6]) */
#define BBREG2_CCAMODE_0                       ((uint8_t)0x00u)
#define BBREG2_CCAMODE_2                       ((uint8_t)0x40u)
#define BBREG2_CCAMODE_1                       ((uint8_t)0x80u)
#define BBREG2_CCAMODE_3                       ((uint8_t)0xC0u)

/* ====================== BBREG3 Register Definitions ======================= */
#define BBREG3_PREVALIDTH3                     ((uint8_t)0x80u)
#define BBREG3_PREVALIDTH2                     ((uint8_t)0x40u)
#define BBREG3_PREVALIDTH1                     ((uint8_t)0x20u)
#define BBREG3_PREVALIDTH0                     ((uint8_t)0x10u)
#define BBREG3_PREDETTH2                       ((uint8_t)0x08u)
#define BBREG3_PREDETTH1                       ((uint8_t)0x04u)
#define BBREG3_PREDETTH0                       ((uint8_t)0x02u)

/* Common presets called out in datasheet */
#define BBREG3_PREVALIDTH_IEEE_802_15_4        ((uint8_t)0xD0u)
#define BBREG3_PREVALIDTH_TURBO                ((uint8_t)0x30u)

/* ====================== BBREG4 Register Definitions ======================= */
#define BBREG4_CSTH2                           ((uint8_t)0x80u)
#define BBREG4_CSTH1                           ((uint8_t)0x40u)
#define BBREG4_CSTH0                           ((uint8_t)0x20u)
#define BBREG4_PRECNT2                         ((uint8_t)0x10u)
#define BBREG4_PRECNT1                         ((uint8_t)0x08u)
#define BBREG4_PRECNT0                         ((uint8_t)0x04u)

/* Common presets called out in datasheet */
#define BBREG4_CSTH_IEEE_802_15_4              ((uint8_t)0x80u)
#define BBREG4_CSTH_TURBO                      ((uint8_t)0x40u)

/* ====================== BBREG6 Register Definitions ======================= */
#define BBREG6_RSSIMODE1                       ((uint8_t)0x80u)
#define BBREG6_RSSIMODE2                       ((uint8_t)0x40u)
#define BBREG6_RSSIRDY                         ((uint8_t)0x01u)

/* ====================== CCAEDTH Register Definitions ====================== */
#define CCAEDTH_CCAEDTH7                       ((uint8_t)0x80u)
#define CCAEDTH_CCAEDTH6                       ((uint8_t)0x40u)
#define CCAEDTH_CCAEDTH5                       ((uint8_t)0x20u)
#define CCAEDTH_CCAEDTH4                       ((uint8_t)0x10u)
#define CCAEDTH_CCAEDTH3                       ((uint8_t)0x08u)
#define CCAEDTH_CCAEDTH2                       ((uint8_t)0x04u)
#define CCAEDTH_CCAEDTH1                       ((uint8_t)0x02u)
#define CCAEDTH_CCAEDTH0                       ((uint8_t)0x01u)

/* LONG ADDRESS CONTROL REGISTER SUMMARY FOR MRF24J40 */
/* ====================== RFCON1 Register Definitions ======================= */
#define RFCON1_VCOOPT7                         ((uint8_t)0x80u)
#define RFCON1_VCOOPT6                         ((uint8_t)0x40u)
#define RFCON1_VCOOPT5                         ((uint8_t)0x20u)
#define RFCON1_VCOOPT4                         ((uint8_t)0x10u)
#define RFCON1_VCOOPT3                         ((uint8_t)0x08u)
#define RFCON1_VCOOPT2                         ((uint8_t)0x04u)
#define RFCON1_VCOOPT1                         ((uint8_t)0x02u)
#define RFCON1_VCOOPT0                         ((uint8_t)0x01u)

/* ====================== TXSTAT Register Definitions ======================= */
#define TXSTAT_TXNRETRY1                       ((uint8_t)0x80u)
#define TXSTAT_TXNRETRY0                       ((uint8_t)0x40u)
#define TXSTAT_CCAFAIL                         ((uint8_t)0x20u)
#define TXSTAT_TXG2FNT                         ((uint8_t)0x10u)
#define TXSTAT_TXG1FNT                         ((uint8_t)0x08u)
#define TXSTAT_TXG2STAT                        ((uint8_t)0x04u)
#define TXSTAT_TXG1STAT                        ((uint8_t)0x02u)
#define TXSTAT_TXNSTAT                         ((uint8_t)0x01u)

/* ====================== RFCON2 Register Definitions ======================= */
#define RFCON2_PLLEN                           ((uint8_t)0x80u)

/* ====================== RFCON3 Register Definitions ======================= */
/* TXPWRL[1:0] (bits [7:6]) */
#define RFCON3_TXPWRL1                         ((uint8_t)0x80u)
#define RFCON3_TXPWRL0                         ((uint8_t)0x40u)

/* TXPWRS[2:0] (bits [5:3]) */
#define RFCON3_TXPWRS2                         ((uint8_t)0x20u)
#define RFCON3_TXPWRS1                         ((uint8_t)0x10u)
#define RFCON3_TXPWRS0                         ((uint8_t)0x08u)

/* Common presets (as in datasheet tables) */
#define RFCON3_TXPWRL_30_DB                    ((uint8_t)0xC0u)
#define RFCON3_TXPWRL_20_DB                    ((uint8_t)0x80u)
#define RFCON3_TXPWRL_10_DB                    ((uint8_t)0x40u)
#define RFCON3_TXPWRS_6P3_DB                   ((uint8_t)0x38u)
#define RFCON3_TXPWRS_4P9_DB                   ((uint8_t)0x30u)
#define RFCON3_TXPWRS_3P7_DB                   ((uint8_t)0x28u)
#define RFCON3_TXPWRS_2P8_DB                   ((uint8_t)0x20u)
#define RFCON3_TXPWRS_1P9_DB                   ((uint8_t)0x18u)
#define RFCON3_TXPWRS_1P2_DB                   ((uint8_t)0x10u)
#define RFCON3_TXPWRS_0P5_DB                   ((uint8_t)0x08u)
#define RFCON3_TXPWRS_0_DB                     ((uint8_t)0x00u)

/* ====================== RFCON5 Register Definitions ======================= */
#define RFCON5_BATTH3                          ((uint8_t)0x80u)
#define RFCON5_BATTH2                          ((uint8_t)0x40u)
#define RFCON5_BATTH1                          ((uint8_t)0x20u)
#define RFCON5_BATTH0                          ((uint8_t)0x10u)

/* Common presets (bits [7:4]) */
#define RFCON5_BATTH_3P5                       ((uint8_t)0xE0u)
#define RFCON5_BATTH_3P3                       ((uint8_t)0xD0u)
#define RFCON5_BATTH_3P2                       ((uint8_t)0xC0u)
#define RFCON5_BATTH_3P1                       ((uint8_t)0xB0u)
#define RFCON5_BATTH_2P8                       ((uint8_t)0xA0u)
#define RFCON5_BATTH_2P7                       ((uint8_t)0x90u)
#define RFCON5_BATTH_2P6                       ((uint8_t)0x80u)
#define RFCON5_BATTH_2P5                       ((uint8_t)0x70u)

/* ====================== RFCON6 Register Definitions ======================= */
#define RFCON6_TXFIL                           ((uint8_t)0x80u)
#define RFCON6_20MRECVR                        ((uint8_t)0x10u)
#define RFCON6_BATEN                           ((uint8_t)0x08u)

/* ====================== RFCON7 Register Definitions ======================= */
#define RFCON7_SLPCLKSEL_100KHZ                ((uint8_t)0x80u)
#define RFCON7_SLPCLKSEL_32KHZ                 ((uint8_t)0x40u)

/* ====================== RFCON8 Register Definitions ======================= */
#define RFCON8_RFVCO                           ((uint8_t)0x10u)

/* ====================== RFSTATE Register Definitions ====================== */
#define RFSTATE_RFSTATE2                       ((uint8_t)0x80u)
#define RFSTATE_RFSTATE1                       ((uint8_t)0x40u)
#define RFSTATE_RFSTATE0                       ((uint8_t)0x20u)

/* Common presets (bits [7:5]) */
#define RFSTATE_RTSEL2                         ((uint8_t)0xE0u)
#define RFSTATE_RTSEL1                         ((uint8_t)0xC0u)
#define RFSTATE_RX                             ((uint8_t)0xA0u)
#define RFSTATE_TX                             ((uint8_t)0x80u)
#define RFSTATE_CALVCO                         ((uint8_t)0x60u)
#define RFSTATE_SLEEP                          ((uint8_t)0x40u)
#define RFSTATE_CALFIL                         ((uint8_t)0x20u)
#define RFSTATE_RESET                          ((uint8_t)0x00u)

/* ====================== SLPCON0 Register Definitions ====================== */
#define SLPCON0_INTEDGE                        ((uint8_t)0x02u)
#define SLPCON0_SLPCLKEN                       ((uint8_t)0x01u)

/* ====================== SLPCON1 Register Definitions ====================== */
#define SLPCON1_CLKOUTEN                       ((uint8_t)0x20u)
#define SLPCON1_SLPCLKDIV4                     ((uint8_t)0x10u)
#define SLPCON1_SLPCLKDIV3                     ((uint8_t)0x08u)
#define SLPCON1_SLPCLKDIV2                     ((uint8_t)0x04u)
#define SLPCON1_SLPCLKDIV1                     ((uint8_t)0x02u)
#define SLPCON1_SLPCLKDIV0                     ((uint8_t)0x01u)

/* ====================== TESTMODE Register Definitions ===================== */
#define TESTMODE_RSSIWAIT1                     ((uint8_t)0x10u)
#define TESTMODE_RSSIWAIT0                     ((uint8_t)0x08u)
#define TESTMODE_TESTMODE2                     ((uint8_t)0x04u)
#define TESTMODE_TESTMODE1                     ((uint8_t)0x02u)
#define TESTMODE_TESTMODE0                     ((uint8_t)0x01u)

/* ====================== Frame Control Field (FCF) ========================= */
/* LSB (Frame Control Field Low Byte) */
#define FCF_LSB_DATA                           ((uint8_t)0x01u)
#define FCF_LSB_ACK                            ((uint8_t)0x02u)
#define FCF_LSB_MAC_COMM                       ((uint8_t)0x03u)
#define FCF_LSB_SECURITY                       ((uint8_t)0x08u)
#define FCF_LSB_FRAME_PEND                     ((uint8_t)0x10u)
#define FCF_LSB_ACK_REQ                        ((uint8_t)0x20u)
#define FCF_LSB_INTRA_PAN                      ((uint8_t)0x40u)

/* MSB (Frame Control Field High Byte) */
#define FCF_MSB_TX_CTR                         ((uint8_t)0x01u)
#define FCF_MSB_TX_CCM128                      ((uint8_t)0x02u)
#define FCF_MSB_TX_CCM64                       ((uint8_t)0x03u)
#define FCF_MSB_TX_CCM32                       ((uint8_t)0x04u)
#define FCF_MSB_TX_CBC_MAC128                  ((uint8_t)0x05u)
#define FCF_MSB_TX_CBC_MAC64                   ((uint8_t)0x06u)
#define FCF_MSB_TX_CBC_MAC32                   ((uint8_t)0x07u)
#define FCF_MSB_SHORT_D_ADD                    ((uint8_t)0x08u)
#define FCF_MSB_LONG_D_ADD                     ((uint8_t)0x0Cu)
#define FCF_MSB_SHORT_S_ADD                    ((uint8_t)0x80u)
#define FCF_MSB_LONG_S_ADD                     ((uint8_t)0xC0u)

#endif /* MRF24J40_REG_H_ */

/** @} */
