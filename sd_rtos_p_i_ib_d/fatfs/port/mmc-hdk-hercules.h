/*
 * mmc-hdk-hercules.h
 *
 *  Created on: 2022. 6. 30.
 *      Author: Kyuyong
 */

#ifndef SD_RTOS_P_I_IB_D_FATFS_PORT_MMC_HDK_HERCULES_H_
#define SD_RTOS_P_I_IB_D_FATFS_PORT_MMC_HDK_HERCULES_H_

#define MODE_POLLING 1
#define MODE_INTERRUPT 2
#define MODE_INTERRUPT_BLOCK 3
#define MODE_DMA 4

#define MODE 4


#include "FreeRTOS.h"
#include "os_task.h"
#include "sys_dma.h"

#include <stdint.h>
#include <stdbool.h>
#include "gio.h"
#include "spi.h"
#include "diskio.h"

void kyuSpiInit(void);


#endif /* SD_RTOS_P_I_IB_D_FATFS_PORT_MMC_HDK_HERCULES_H_ */
