/*
 * k_dma.c
 *
 *  Created on: 2022. 5. 23.
 *      Author: Kyuyong
 */
#include "sys_dma.h"
#include "k_dma.h"


void dmaConfigCtrlTxTxPacket(uint32 sadd, uint32 dadd, uint16 ElmntCnt, uint16 FrameCnt)
{
    g_dmaCTRLPKT_TX_TX.SADD      = sadd;               /* source address             */
    g_dmaCTRLPKT_TX_TX.DADD      = dadd;               /* destination  address       */
    g_dmaCTRLPKT_TX_TX.CHCTRL    = 0;                  /* channel control            */
    g_dmaCTRLPKT_TX_TX.FRCNT     = FrameCnt;           /* frame count                */
    g_dmaCTRLPKT_TX_TX.ELCNT     = ElmntCnt;           /* element count              */
    g_dmaCTRLPKT_TX_TX.ELDOFFSET = 0;                  /* element destination offset */
    g_dmaCTRLPKT_TX_TX.ELSOFFSET = 1;                  /* element destination offset */
    g_dmaCTRLPKT_TX_TX.FRDOFFSET = 0;                  /* frame destination offset   */
    g_dmaCTRLPKT_TX_TX.FRSOFFSET = 0;                  /* frame destination offset   */
    g_dmaCTRLPKT_TX_TX.PORTASGN  = 4;                  /* port b                     */
    g_dmaCTRLPKT_TX_TX.RDSIZE    = ACCESS_8_BIT;      /* read size                  */
    g_dmaCTRLPKT_TX_TX.WRSIZE    = ACCESS_8_BIT;      /* write size                 */
    g_dmaCTRLPKT_TX_TX.TTYPE     = FRAME_TRANSFER;     /* transfer type              */
    g_dmaCTRLPKT_TX_TX.ADDMODERD = ADDR_INC1;         /* address mode read          */
    g_dmaCTRLPKT_TX_TX.ADDMODEWR = ADDR_FIXED;          /* address mode write         */
    g_dmaCTRLPKT_TX_TX.AUTOINIT  = AUTOINIT_OFF;        /* autoinit                   */
}

void dmaConfigCtrlTxRxPacket(uint32 sadd, uint32 dadd, uint16 ElmntCnt, uint16 FrameCnt)
{
    g_dmaCTRLPKT_TX_RX.SADD      = sadd;               /* source address             */
    g_dmaCTRLPKT_TX_RX.DADD      = dadd;               /* destination  address       */
    g_dmaCTRLPKT_TX_RX.CHCTRL    = 0;                  /* channel control            */
    g_dmaCTRLPKT_TX_RX.FRCNT     = FrameCnt;           /* frame count                */
    g_dmaCTRLPKT_TX_RX.ELCNT     = ElmntCnt;           /* element count              */
    g_dmaCTRLPKT_TX_RX.ELDOFFSET = 0;                  /* element destination offset */
    g_dmaCTRLPKT_TX_RX.ELSOFFSET = 0;                  /* element destination offset */
    g_dmaCTRLPKT_TX_RX.FRDOFFSET = 0;                  /* frame destination offset   */
    g_dmaCTRLPKT_TX_RX.FRSOFFSET = 0;                  /* frame destination offset   */
    g_dmaCTRLPKT_TX_RX.PORTASGN  = 4;                  /* port b                     */
    g_dmaCTRLPKT_TX_RX.RDSIZE    = ACCESS_8_BIT;      /* read size                  */
    g_dmaCTRLPKT_TX_RX.WRSIZE    = ACCESS_8_BIT;      /* write size                 */
    g_dmaCTRLPKT_TX_RX.TTYPE     = FRAME_TRANSFER;     /* transfer type              */
    g_dmaCTRLPKT_TX_RX.ADDMODERD = ADDR_FIXED;         /* address mode read          */
    g_dmaCTRLPKT_TX_RX.ADDMODEWR = ADDR_FIXED;          /* address mode write         */
    g_dmaCTRLPKT_TX_RX.AUTOINIT  = AUTOINIT_OFF;        /* autoinit                   */
}

void dmaConfigCtrlRxTxPacket(uint32 sadd, uint32 dadd, uint16 ElmntCnt, uint16 FrameCnt)
{
    g_dmaCTRLPKT_RX_TX.SADD      = sadd;               /* source address             */
    g_dmaCTRLPKT_RX_TX.DADD      = dadd;               /* destination  address       */
    g_dmaCTRLPKT_RX_TX.CHCTRL    = 0;                  /* channel control            */ //Next channel triggering *chaining
    g_dmaCTRLPKT_RX_TX.FRCNT     = FrameCnt;           /* frame count                */ //FrameCnt == 511
    g_dmaCTRLPKT_RX_TX.ELCNT     = ElmntCnt;           /* element count              */ //ElementCnt = 1
    g_dmaCTRLPKT_RX_TX.ELDOFFSET = 0;                  /* element destination offset */
    g_dmaCTRLPKT_RX_TX.ELSOFFSET = 0;                  /* element destination offset */
    g_dmaCTRLPKT_RX_TX.FRDOFFSET = 0;                  /* frame destination offset   */
    g_dmaCTRLPKT_RX_TX.FRSOFFSET = 0;                  /* frame destination offset   */
    g_dmaCTRLPKT_RX_TX.PORTASGN  = 4;                  /* port b                     */
    g_dmaCTRLPKT_RX_TX.RDSIZE    = ACCESS_8_BIT;      /* read size                  */
    g_dmaCTRLPKT_RX_TX.WRSIZE    = ACCESS_8_BIT;      /* write size                 */
    g_dmaCTRLPKT_RX_TX.TTYPE     = FRAME_TRANSFER;     /* transfer type              */
    g_dmaCTRLPKT_RX_TX.ADDMODERD = ADDR_FIXED;         /* address mode read          */
    g_dmaCTRLPKT_RX_TX.ADDMODEWR = ADDR_FIXED;          /* address mode write         */
    g_dmaCTRLPKT_RX_TX.AUTOINIT  = AUTOINIT_OFF;        /* autoinit                   */
}

void dmaConfigCtrlRxRxPacket(uint32 sadd, uint32 dadd, uint16 ElmntCnt, uint16 FrameCnt)
{
    g_dmaCTRLPKT_RX_RX.SADD      = sadd;               /* source address             */
    g_dmaCTRLPKT_RX_RX.DADD      = dadd;               /* destination  address       */
    g_dmaCTRLPKT_RX_RX.CHCTRL    = 0;                  /* channel control            */
    g_dmaCTRLPKT_RX_RX.FRCNT     = FrameCnt;           /* frame count                */
    g_dmaCTRLPKT_RX_RX.ELCNT     = ElmntCnt;           /* element count              */
    g_dmaCTRLPKT_RX_RX.ELDOFFSET = 1;                  /* element destination offset */
    g_dmaCTRLPKT_RX_RX.ELSOFFSET = 0;                  /* element destination offset */
    g_dmaCTRLPKT_RX_RX.FRDOFFSET = 0;                  /* frame destination offset   */
    g_dmaCTRLPKT_RX_RX.FRSOFFSET = 0;                  /* frame destination offset   */
    g_dmaCTRLPKT_RX_RX.PORTASGN  = 4;                  /* port b                     */
    g_dmaCTRLPKT_RX_RX.RDSIZE    = ACCESS_8_BIT;      /* read size                  */
    g_dmaCTRLPKT_RX_RX.WRSIZE    = ACCESS_8_BIT;      /* write size                 */
    g_dmaCTRLPKT_RX_RX.TTYPE     = FRAME_TRANSFER;     /* transfer type              */
    g_dmaCTRLPKT_RX_RX.ADDMODERD = ADDR_FIXED;         /* address mode read          */
    g_dmaCTRLPKT_RX_RX.ADDMODEWR = ADDR_INC1;          /* address mode write         */
    g_dmaCTRLPKT_RX_RX.AUTOINIT  = AUTOINIT_OFF;        /* autoinit                   */
}
