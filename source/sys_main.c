/** @file sys_main.c 
*   @brief Application main file
*   @date 11-Dec-2018
*   @version 04.07.01
*
*   This file contains an empty main function,
*   which can be used for the application.
*/

/* 
* Copyright (C) 2009-2018 Texas Instruments Incorporated - www.ti.com 
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* USER CODE BEGIN (0) */
/* USER CODE END */

/* Include Files */

#include "sys_common.h"

/* USER CODE BEGIN (1) */
#include "FreeRTOS.h"
#include "os_task.h"
#include "uartstdio.h"
#include "diskio.h"
#include "mmc-hdk-hercules.h"

#include "spi.h"
#include "sci.h"
/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */

extern int cmdMount();
extern int Cmd_ls(int argc, char *argv[]);
extern int cmdWrite(char* writeFileName, void* bufToWrite, int bytesToWrite, int overWrite);

void vSDTimerTask(void *pvParameters)
{
    int i = 0 ;
    while(1)
    {
        if(i < 10)
        {
            i++;
        }
        else
        {
            i++;
            i = i % 20;
        }
        disk_timerproc();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

#define APPENDMODE 0
#define NONAPPENDMODE 1
int dummy1 = 0;
char *dummyargv[];
char a[4096];

void vSDTestTask(void *pvParameters)
{
    cmdMount();
    char path[50] = "BLKtest.txt";

    int i = 0;
    for(i = 0 ; i < 4096 ; i++)
    {
        a[i] = 'a';
    }
    i = 0;
    for(i = 0 ; i < 100 ; i++)
    {
        //Cmd_ls(dummy1, dummyargv);
        while(0 != cmdWrite(path,a,sizeof(a),NONAPPENDMODE));
        //Cmd_ls(dummy1, dummyargv);
    }

    Cmd_ls(dummy1,dummyargv);

    while(1)
    {


    }
}

xTaskHandle xSDTimerTaskHandle;
xTaskHandle xSDTestTaskHandle;


/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */


    sciInit();
    spiInit();
    kyuSpiInit();

    UARTprintf("System Ready to Start!\r\n");

    xTaskCreate(vSDTimerTask,"SDTimerTask",configMINIMAL_STACK_SIZE,NULL,5,&xSDTimerTaskHandle);
    #if MODE != MODE_DMA
    xTaskCreate(vSDTestTask,"SDTestTask",configMINIMAL_STACK_SIZE,NULL,4,&xSDTestTaskHandle);
    #else
    xTaskCreate(vSDTestTask,"SDTestTask",configMINIMAL_STACK_SIZE,NULL,4 | portPRIVILEGE_BIT ,&xSDTestTaskHandle);  // DMA REGs need privilege mode.
    #endif

    vTaskStartScheduler();

    while(1);
/* USER CODE END */

    return 0;
}


/* USER CODE BEGIN (4) */
/* USER CODE END */
