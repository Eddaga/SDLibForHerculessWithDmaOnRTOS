/*
 * k_dma.h
 *
 *  Created on: 2022. 5. 23.
 *      Author: Kyuyong
 */

#ifndef FATFS_PORT_DMAFORSDCARD_K_DMA_H_
#define FATFS_PORT_DMAFORSDCARD_K_DMA_H_

#include "sys_dma.h"
void dmaConfigCtrlTxTxPacket(uint32 sadd, uint32 dadd, uint16 ElmntCnt, uint16 FrameCnt);
void dmaConfigCtrlTxRxPacket(uint32 sadd, uint32 dadd, uint16 ElmntCnt, uint16 FrameCnt);
void dmaConfigCtrlRxTxPacket(uint32 sadd, uint32 dadd, uint16 ElmntCnt, uint16 FrameCnt);
void dmaConfigCtrlRxRxPacket(uint32 sadd, uint32 dadd, uint16 ElmntCnt, uint16 FrameCnt);

g_dmaCTRL g_dmaCTRLPKT_TX_TX,
          g_dmaCTRLPKT_TX_RX,
          g_dmaCTRLPKT_RX_TX,
          g_dmaCTRLPKT_RX_RX;

#endif /* FATFS_PORT_DMAFORSDCARD_K_DMA_H_ */
