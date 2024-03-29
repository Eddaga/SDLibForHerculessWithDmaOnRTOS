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
#include "string.h"
#include "sys_core.h"
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
extern int speedTestOpen(char* writeFileName, int overWrite);
extern int speedTestWrite(char* bufToWrite, int bytesToWrite, unsigned int* ui32BytesWrite );
extern int speedTestClose();

void vSDTimerTask(void *pvParameters)
{
    spiREG3->PC3 &= ~(0x02);
    while(1)
    {

        disk_timerproc();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

}

#define APPENDMODE 0
#define NONAPPENDMODE 1
int dummy1 = 0;
char *dummyargv[];
char a[2048];
char path[20] = "SPEEDTES.TXT";

//void vSDTestTask(void *pvParameters)
//{
//    Cmd_ls(dummy1,dummyargv);
//    while(1)
//    {
//        while(0 != cmdWrite(path,a,sizeof(a),NONAPPENDMODE));
//    }
//
//}

void vSDTestTask(void *pvParameters)
{
    int i  = 0;
    unsigned int writtenBytesNum;
    //fTickType_t start, done;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 50;







    speedTestOpen(path,NONAPPENDMODE);



    xLastWakeTime = xTaskGetTickCount();


    while(i < 4)
    {
        //start = xTaskGetTickCount();

        //spiREG3->PC3 = spiREG3->PC3 ^ (1 << 1);
        while(0 != speedTestWrite(a,sizeof(a),&writtenBytesNum));

        //done = xTaskGetTickCount();
        //UARTprintf("%d, %d, %d ms\r\n",(int)start, (int)done, (int)pdMS_TO_TICKS(done-start));

        i++;

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }





    speedTestClose();
    //spiREG3->PC3 &= ~(0x02);
    spiREG3->PC3 |= 0x02;
    //Cmd_ls(dummy1,dummyargv);
    while(1);

}

//void vHighestTask(void *pvParameters)
//{
//    while(1)
//    {
//
//    }
//}


xTaskHandle xSDTimerTaskHandle;
xTaskHandle xSDTestTaskHandle;
//xTaskHandle xHightestTaskHandle;

/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
    int i = 0;
    for(i = 0 ; i < sizeof(a) ; i++)
    {
        a[i] = 'a';
    }

    sciInit();
    gioInit();
    spiInit();


    //kyuSpiInit();
    _enable_interrupt_();
    cmdMount();

    UARTprintf("System Ready to Start!\r\n");



    xTaskCreate(vSDTimerTask,"SDTimerTask",configMINIMAL_STACK_SIZE,NULL,3,&xSDTimerTaskHandle);
    #if MODE != MODE_DMA
    //xTaskCreate(vSDTestTask,"SDTestTask",configMINIMAL_STACK_SIZE,NULL,3,&xSDTestTaskHandle);
    xTaskCreate(vSDTestTask,"SDTestTask",configMINIMAL_STACK_SIZE,NULL,1 | portPRIVILEGE_BIT,&xSDTestTaskHandle);
    #else
    xTaskCreate(vSDTestTask,"SDTestTask",configMINIMAL_STACK_SIZE,NULL,1 | portPRIVILEGE_BIT ,&xSDTestTaskHandle);  // DMA REGs need privilege mode.
    #endif
    //spiREG3->PC3 |= 0x02;
    vTaskStartScheduler();

    while(1);
/* USER CODE END */

    return 0;
}


/* USER CODE BEGIN (4) */
/* USER CODE END */
