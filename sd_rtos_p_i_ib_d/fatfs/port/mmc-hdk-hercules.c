/*-----------------------------------------------------------------------*/
/* MMC/SDC (in SPI mode) control module  (C)ChaN, 2007                   */
/*-----------------------------------------------------------------------*/
/* Only rcvr_spi(), xmit_spi(), disk_timerproc() and some macros         */
/* are platform dependent.                                               */
/*-----------------------------------------------------------------------*/

/*
 * This file was modified from a sample available from the FatFs
 * web site. It was modified to work with an HDK development
 * board.
 */

#include "FreeRTOS.h"
#include "os_task.h"
#include "sys_dma.h"

#include <stdint.h>
#include <stdbool.h>
#include "gio.h"
#include "spi.h"
#include "diskio.h"


#include "mmc-hdk-hercules.h"
#include "k_dma.h"
#include "sys_dma.h"

/* Definitions for MMC/SDC command */
#define CMD0    (0x40+0)    /* GO_IDLE_STATE */
#define CMD1    (0x40+1)    /* SEND_OP_COND */
#define CMD8    (0x40+8)    /* SEND_IF_COND */
#define CMD9    (0x40+9)    /* SEND_CSD */
#define CMD10    (0x40+10)    /* SEND_CID */
#define CMD12    (0x40+12)    /* STOP_TRANSMISSION */
#define CMD16    (0x40+16)    /* SET_BLOCKLEN */
#define CMD17    (0x40+17)    /* READ_SINGLE_BLOCK */
#define CMD18    (0x40+18)    /* READ_MULTIPLE_BLOCK */
#define CMD23    (0x40+23)    /* SET_BLOCK_COUNT */
#define CMD24    (0x40+24)    /* WRITE_BLOCK */
#define CMD25    (0x40+25)    /* WRITE_MULTIPLE_BLOCK */
#define CMD41    (0x40+41)    /* SEND_OP_COND (ACMD) */
#define CMD55    (0x40+55)    /* APP_CMD */
#define CMD58    (0x40+58)    /* READ_OCR */

// SSI port
#define SDC_SSI_BASE            SSI0_BASE
#define SDC_SSI_SYSCTL_PERIPH   SYSCTL_PERIPH_SSI0

// GPIO for SSI pins
#define SDC_GPIO_PORT_BASE      GPIO_PORTA_BASE
#define SDC_GPIO_SYSCTL_PERIPH  SYSCTL_PERIPH_GPIOA
#define SDC_SSI_CLK             GPIO_PIN_2
#define SDC_SSI_TX              GPIO_PIN_5
#define SDC_SSI_RX              GPIO_PIN_4
#define SDC_SSI_FSS             GPIO_PIN_3
#define SDC_SSI_PINS            (SDC_SSI_TX | SDC_SSI_RX | SDC_SSI_CLK |      \
                                 SDC_SSI_FSS)

// asserts the CS pin to the card

void kyuSpiInit(void)
{
    #if MODE == MODE_POLLING

    spiREG3->GCR1 = spiREG3->GCR1 & 0xFEFFFFFFU;
    spiREG3->INT0 = spiREG3->INT0 & 0xFFFFFEFFU;
    spiREG3->GCR1 = (spiREG3->GCR1 & 0xFEFFFFFFU) | 0x01000000U;

    #else   // except POLLING MODE, all MODEs use spi interrupt.

    spiREG3->GCR1 = spiREG3->GCR1 & 0xFEFFFFFFU;
    spiREG3->INT0 = (spiREG3->INT0 & 0xFFFF0000U)
                  | (uint32)((uint32)0U << 9U)  /* TXINT */
                  | (uint32)((uint32)1U << 8U)  /* RXINT */
                  | (uint32)((uint32)0U << 6U)  /* OVRNINT */
                  | (uint32)((uint32)0U << 4U)  /* BITERR */
                  | (uint32)((uint32)0U << 3U)  /* DESYNC */
                  | (uint32)((uint32)0U << 2U)  /* PARERR */
                  | (uint32)((uint32)0U << 1U) /* TIMEOUT */
                  | (uint32)((uint32)0U << 0U);  /* DLENERR */
    spiREG3->GCR1 = (spiREG3->GCR1 & 0xFEFFFFFFU) | 0x01000000U;

    #endif
}


static
void DESELECT (void)
{
    spiREG3->PC3 |=  0x01;    
}

// de-asserts the CS pin to the card
static
void SELECT (void)
{
    spiREG3->PC3 &= ~0x01;    
}


/*------------------------------------------------------------------------------
  Write and Read a byte on SPI interface
 *------------------------------------------------------------------------------*/

#if MODE == MODE_POLLING
unsigned char SPI_send (unsigned char outb) {
    
  while ((spiREG3->FLG & 0x0200) == 0); // Wait until TXINTFLG is set for previous transmission
  spiREG3->DAT1 = outb | 0x10010000;    // transmit register address    

  while ((spiREG3->FLG & 0x0100) == 0); // Wait until RXINTFLG is set when new value is received
  return((unsigned char)spiREG3->BUF);  // Return received value
}

#else

unsigned char SPI_send (unsigned char outb)
{
    unsigned int Buf;

    setTaskHandlerForISR(xTaskGetCurrentTaskHandle());

    spiREG3->DAT1 = outb | 0x10010000;     
    xTaskNotifyWait(0,0,&Buf,portMAX_DELAY);  // Notification으로 값이 올 때 까지 대기.
    
    return (unsigned char)Buf;
}
#endif

/*--------------------------------------------------------------------------

   Module Private Functions

---------------------------------------------------------------------------*/

static volatile
DSTATUS Stat = STA_NOINIT;    /* Disk status */

static volatile
BYTE Timer1, Timer2;    /* 100Hz decrement timer */

static
BYTE CardType;            /* b0:MMC, b1:SDC, b2:Block addressing */

static
BYTE PowerFlag = 0;     /* indicates if "power" is on */

/*-----------------------------------------------------------------------*/
/* Transmit a byte to MMC via SPI  (Platform dependent)                  */
/*-----------------------------------------------------------------------*/

#if MODE == MODE_POLLING
static void xmit_spi(BYTE dat)
{
	unsigned int ui32RcvDat;
    
    while ((spiREG3->FLG & 0x0200) == 0); // Wait until TXINTFLG is set for previous transmission   
    spiREG3->DAT1 = dat | 0x10010000;    // transmit register address

    while ((spiREG3->FLG & 0x0100) == 0); // Wait until RXINTFLG is set when new value is received  
    ui32RcvDat = spiREG3->BUF;  // to get received value
}
#else

static void xmit_spi (BYTE dat)
{
    unsigned int trashcan;
    setTaskHandlerForISR(xTaskGetCurrentTaskHandle());
    spiREG3->DAT1 = dat | 0x10010000;     // write data to spi3.
    xTaskNotifyWait(0,0,&trashcan,portMAX_DELAY);  // wait until Notification come.
}
#endif

/*-----------------------------------------------------------------------*/
/* Receive a byte from MMC via SPI  (Platform dependent)                 */
/*-----------------------------------------------------------------------*/

#if MODE == MODE_POLLING
static
BYTE rcvr_spi (void)
{
    while ((spiREG3->FLG & 0x0200) == 0); // Wait until TXINTFLG is set for previous transmission
    spiREG3->DAT1 = 0xFF | 0x10010000;    // transmit register address

    while ((spiREG3->FLG & 0x0100) == 0); // Wait until RXINTFLG is set when new value is received
    return((unsigned char)spiREG3->BUF);  // Return received value
}
#else

BYTE rcvr_spi (void)
{
    unsigned int Rcvbuf;
    setTaskHandlerForISR(xTaskGetCurrentTaskHandle());
    spiREG3->DAT1 = 0xFF | 0x10010000;     // write data to spi3.
    xTaskNotifyWait(0,0,&Rcvbuf,portMAX_DELAY);  // wait until Notification come. 
    return (unsigned char)Rcvbuf;    // after, return notification value.
}
#endif

static
void rcvr_spi_m (BYTE *dst)
{
    *dst = rcvr_spi();
}

/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------------------------------------------------*/

static
BYTE wait_ready (void)
{
    BYTE res;


    Timer2 = 50;    /* Wait for ready in timeout of 500ms */
    rcvr_spi();
    do
        res = rcvr_spi();
    while ((res != 0xFF) && Timer2);

    return res;
}

/*-----------------------------------------------------------------------*/
/* Send 80 or so clock transitions with CS and DI held high. This is     */
/* required after card power up to get it into SPI mode                  */
/*-----------------------------------------------------------------------*/
static
void send_initial_clock_train(void)
{
    unsigned int i;

    /* Ensure CS is held high. */
    DESELECT();

    /* Send 10 bytes over the SSI. This causes the clock to wiggle the */
    /* required number of times. */
    for(i = 0 ; i < 10 ; i++)
    {
        /* Write DUMMY data */
        /* FIFO. */
    	SPI_send (0xFF);
    }
}

/*-----------------------------------------------------------------------*/
/* Power Control  (Platform dependent)                                   */
/*-----------------------------------------------------------------------*/
/* When the target system does not support socket power control, there   */
/* is nothing to do in these functions and chk_power always returns 1.   */

static
void power_on (void)
{
    /*
     * This doesn't really turn the power on, but initializes the
     * SPI port and pins needed to talk to the card.
     */
	spiInit();
	kyuSpiInit();
    /* Set DI and CS high and apply more than 74 pulses to SCLK for the card */
    /* to be able to accept a native command. */
    send_initial_clock_train();

    PowerFlag = 1;
}

// set the SPI speed to the max setting
static
void set_max_speed(void)
{
	  spiREG3->FMT0 &= 0xFFFF00FF;   // mask out baudrate prescaler
	                                 // Max. 5 MBit used for Data Transfer.
      spiREG3->FMT0 |= 0 << 8;     // baudrate prescale 10MHz / (1+1) = 5MBit
}

static
void power_off (void)
{
    PowerFlag = 0;
}

static
int chk_power(void)        /* Socket power state: 0=off, 1=on */
{
    return PowerFlag;
}



/*-----------------------------------------------------------------------*/
/* Receive a data packet from MMC                                        */
/*-----------------------------------------------------------------------*/

static
bool rcvr_datablock (
    BYTE *buff,            /* Data buffer to store received data */
    UINT btr            /* Byte count (must be even number) */
)
{
    BYTE token;


    Timer1 = 100;
    do {                            /* Wait for data packet in timeout of 100ms */
        token = rcvr_spi();
    } while ((token == 0xFF) && Timer1);
    if(token != 0xFE) return FALSE;    /* If not valid data token, retutn with error */

    do {                            /* Receive the data block into buffer */
        rcvr_spi_m(buff++);
        rcvr_spi_m(buff++);
    } while (btr -= 2);
    rcvr_spi();                        /* Discard CRC */
    rcvr_spi();

    return TRUE;                    /* Return with success */
}
#if MODE == MODE_INTERRUPT_BLOCK

const char *BlockRxBufPointer;
int RxCnt = 0;
extern int SDMode;

static rx_blockSPI(const char *recvBuffer)
{
   //이 함수 역시 512Bytes만큼 교환을 하는 함수
   SDMode = LargeRxMode;
   RxCnt = 0;
   BlockRxBufPointer = recvBuffer;
   spiREG3->DAT1 = 0xFF | 0x10010000;

   xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
   SDMode = SINGLEMODE;
}

static bool rcvr_largedatablock (    BYTE *buff            /* Data buffer to store received data */)
{
    BYTE token;
    Timer1 = 100;
    do {                            /* Wait for data packet in timeout of 100ms */
        token = rcvr_spi();
    } while ((token == 0xFF) && Timer1);
    if(token != 0xFE) return FALSE;    /* If not valid data token, retutn with error */

    rx_blockSPI((const char*)buff);

    rcvr_spi();                        /* Discard CRC */
    rcvr_spi();

    return TRUE;                    /* Return with success */
}






#elif MODE == MODE_DMA

char *BlockRxBufPointer;
int RxCnt = 0;
extern int SDMode;


static rx_blockSPI(char *recvBuffer)
{
    char xmitDataForRx = 0xFF;
    setTaskHandlerForISRinDMA(xTaskGetCurrentTaskHandle());
    // set DMA reg.

    // channel 0 <- ReqLine 14: SPI receive standard SPI
    // channel 1 <- ReqLine 15: SPI transmit standard SPI
    dmaReqAssign(1,15);  // TXRX as CH15. SPI3 transmit is DMAREQ0
    dmaReqAssign(0,14);  // TXTX as CH14. SPI3 receive is DMAREQ1

    // change spi interrupt spirx to DMA
    spiREG3->INT0 &= ~(uint32)((uint32)1U << 8U);  /* RXINT OFF*/
    spiREG3->INT0 |= (uint32)((uint32)1U << 16U);   /* DMAINT ON*/

    dmaConfigCtrlRxTxPacket((uint32)&xmitDataForRx,(uint32)(&(spiREG3->DAT1)),1,512);
    dmaConfigCtrlRxRxPacket( (uint32)(&(spiREG3->BUF)),(uint32)recvBuffer,1,512);

    dmaSetCtrlPacket(1,g_dmaCTRLPKT_RX_TX);
    dmaSetCtrlPacket(0,g_dmaCTRLPKT_RX_RX);

    dmaSetChEnable(1,DMA_HW);
    dmaSetChEnable(0,DMA_HW);

    // enable DMA interrupt
    dmaEnableInterrupt(0, BTC);
    // start DMA
    dmaEnable();

    // block until notification come

    xTaskNotifyWait(0,0,NULL,portMAX_DELAY);


    dmaDisable();
    dmaDisableInterrupt(0, BTC);

    // change spi interrupt DMA to spirx
    spiREG3->INT0 |= (uint32)((uint32)1U << 8U);  /* RXINT ON*/
    spiREG3->INT0 &= ~(uint32)((uint32)1U << 16U);   /* DMAINT OFF*/
}

static bool rcvr_largedatablock (    BYTE *buff            /* Data buffer to store received data */)
{
    BYTE token;
    Timer1 = 100;
    do {                            /* Wait for data packet in timeout of 100ms */
        token = rcvr_spi();
    } while ((token == 0xFF) && Timer1);
    if(token != 0xFE) return FALSE;    /* If not valid data token, retutn with error */

    rx_blockSPI(buff);

    rcvr_spi();                        /* Discard CRC */
    rcvr_spi();

    return TRUE;                    /* Return with success */
}


#endif


/*-----------------------------------------------------------------------*/
/* Send a data packet to MMC                                             */
/*-----------------------------------------------------------------------*/

// xmit_datablock only need for 512byte sending. so at int_block mode and dma mode, this function doesn't need to comfile.
#if _READONLY == 0
    #if MODE == MODE_POLLING | MODE == MODE_INTERRUPT   

        static
        bool xmit_datablock (
            const BYTE *buff,    /* 512 byte data block to be transmitted */
            BYTE token            /* Data/Stop token */
        )
        {
            BYTE resp, wc;


            if (wait_ready() != 0xFF) return FALSE;

            xmit_spi(token);                    /* Xmit data token */
            if (token != 0xFD) {    /* Is data token */
                wc = 0;
                do {                            /* Xmit the 512 byte data block to MMC */
                    xmit_spi(*buff++);
                    xmit_spi(*buff++);
                } while (--wc);
                xmit_spi(0xFF);                    /* CRC (Dummy) */
                xmit_spi(0xFF);
                resp = rcvr_spi();                /* Reveive data response */
                if ((resp & 0x1F) != 0x05)        /* If not accepted, return with error */
                    return FALSE;
            }

            return TRUE;
        }

    #elif MODE == MODE_INTERRUPT_BLOCK

        const char *BlockTxBufPointer;
        int TxCnt = 0;

        static tx_blockSPI(const char *sendBuffer)
        {
            //SPI block transfer
            SDMode = LargeTxMode;
            TxCnt = 0;
            BlockTxBufPointer = sendBuffer;
            spiREG3->DAT1 = BlockTxBufPointer[0] | 0x10010000;
            xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
            SDMode = SINGLEMODE;
        }

        static bool xmit_largedatablock (
        const BYTE *buff,    /* 512 byte data block to be transmitted */
        BYTE token            /* Data/Stop token */
        )
        {
        BYTE resp;

        if (wait_ready() != 0xFF) return FALSE;

        xmit_spi(token);                    /* Xmit data token */
        if (token != 0xFD)
        {    /* Is data token */

            //here: SPI blockTransfer
            //int count = 0;

            tx_blockSPI((const char*) buff);

            xmit_spi(0xFF);                    /* CRC (Dummy) */
            xmit_spi(0xFF);
            resp = rcvr_spi();                /* Reveive data response */
            if ((resp & 0x1F) != 0x05)        /* If not accepted, return with error */
                return FALSE;
        }

        return TRUE;
        }


    #elif MODE == MODE_DMA

        char *BlockTxBufPointer;
        int TxCnt = 0;


        static tx_blockSPI(char *sendBuffer)
        {
            char Rxbuf = 0x00;
            setTaskHandlerForISRinDMA(xTaskGetCurrentTaskHandle());

            // set DMA reg.
            dmaReqAssign(1,15);  // TXTX as CH1. SPI3 transmit is DMAREQ1
            dmaReqAssign(0,14);  // TXRX as CH0. SPI3 receive is DMAREQ0

            // change spi interrupt spirx to DMA
            spiREG3->INT0 &= ~(uint32)((uint32)1U << 8U);  /* RXINT OFF*/
            spiREG3->INT0 |= (uint32)((uint32)1U << 16U);   /* DMAINT ON*/

            dmaConfigCtrlTxTxPacket((uint32)sendBuffer,(uint32)(&(spiREG3->DAT1)),1,512);
            dmaConfigCtrlTxRxPacket((uint32)(&(spiREG3->BUF)),(uint32)&Rxbuf,1,512);

            dmaSetCtrlPacket(1,g_dmaCTRLPKT_TX_TX);
            dmaSetCtrlPacket(0,g_dmaCTRLPKT_TX_RX);

            dmaSetChEnable(1,DMA_HW);
            dmaSetChEnable(0,DMA_HW);

            // enable DMA interrupt
            dmaEnableInterrupt(0, BTC);

            // start DMA

            dmaEnable();
            //spiREG
            // block until notification come

            xTaskNotifyWait(0,0,NULL,portMAX_DELAY);

            // stop DMA
            dmaDisable();

            // disable DMA interrupt
            dmaDisableInterrupt(0, BTC);

            // change spi interrupt DMA to spirx
            spiREG3->INT0 |= (uint32)((uint32)1U << 8U);  /* RXINT OFF*/
            spiREG3->INT0 &= ~(uint32)((uint32)1U << 16U);   /* DMAINT ON*/
        }    

        static
        bool xmit_largedatablock (
            const BYTE *buff,    /* 512 byte data block to be transmitted */
            BYTE token            /* Data/Stop token */
        )
        {
            BYTE resp;
            if (wait_ready() != 0xFF) return FALSE;

            xmit_spi(token);                    /* Xmit data token */
            if (token != 0xFD)
            {    /* Is data token */

                //here: SPI blockTransfer


                tx_blockSPI(buff);

                xmit_spi(0xFF);                    /* CRC (Dummy) */
                xmit_spi(0xFF);
                resp = rcvr_spi();                /* Reveive data response */
                if ((resp & 0x1F) != 0x05)        /* If not accepted, return with error */
                    return FALSE;
            }
            return TRUE;
        }

    #endif
#endif /* _READONLY */



/*-----------------------------------------------------------------------*/
/* Send a command packet to MMC                                          */
/*-----------------------------------------------------------------------*/

static
BYTE send_cmd (
    BYTE cmd,        /* Command byte */
    DWORD arg        /* Argument */
)
{
    BYTE n, res;


    if (wait_ready() != 0xFF) return 0xFF;

    /* Send command packet */
    xmit_spi(cmd);                        /* Command */
    xmit_spi((BYTE)(arg >> 24));        /* Argument[31..24] */
    xmit_spi((BYTE)(arg >> 16));        /* Argument[23..16] */
    xmit_spi((BYTE)(arg >> 8));            /* Argument[15..8] */
    xmit_spi((BYTE)arg);                /* Argument[7..0] */
    n = 0xff;
    if (cmd == CMD0) n = 0x95;            /* CRC for CMD0(0) */
    if (cmd == CMD8) n = 0x87;            /* CRC for CMD8(0x1AA) */
    xmit_spi(n);

    /* Receive command response */
    if (cmd == CMD12) rcvr_spi();        /* Skip a stuff byte when stop reading */
    n = 10;                                /* Wait for a valid response in timeout of 10 attempts */
    do
        res = rcvr_spi();
    while ((res & 0x80) && --n);

    return res;            /* Return with the response value */
}

/*-----------------------------------------------------------------------*
 * Send the special command used to terminate a multi-sector read.
 *
 * This is the only command which can be sent while the SDCard is sending
 * data. The SDCard spec indicates that the data transfer will stop 2 bytes
 * after the 6 byte CMD12 command is sent and that the card will then send
 * 0xFF for between 2 and 6 more bytes before the R1 response byte.  This
 * response will be followed by another 0xFF byte.  In testing, however, it
 * seems that some cards don't send the 2 to 6 0xFF bytes between the end of
 * data transmission and the response code.  This function, therefore, merely
 * reads 10 bytes and, if the last one read is 0xFF, returns the value of the
 * latest non-0xFF byte as the response code.
 *
 *-----------------------------------------------------------------------*/

static
BYTE send_cmd12 (void)
{
    BYTE n, res, val;

    /* For CMD12, we don't wait for the card to be idle before we send
     * the new command.
     */

    /* Send command packet - the argument for CMD12 is ignored. */
    xmit_spi(CMD12);
    xmit_spi(0);
    xmit_spi(0);
    xmit_spi(0);
    xmit_spi(0);

    xmit_spi(0xFF); // this should be 0xFF. all cmd should be end with this signal. 22.06.29. kyu changed.

    /* Read up to 10 bytes from the card, remembering the value read if it's
       not 0xFF */
    for(n = 0; n < 10; n++)
    {
        val = rcvr_spi();
        if(val != 0xFF)
        {
            res = val;
        }
    }

    return res;            /* Return with the response value */
}

/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
    BYTE drv        /* Physical drive nmuber (0) */
)
{
    BYTE n, ty, ocr[4];


    if (drv) return STA_NOINIT;            /* Supports only single drive */
    if (Stat & STA_NODISK) return Stat;    /* No card in the socket */

    power_on();                            /* Force socket power on */
    send_initial_clock_train();            /* Ensure the card is in SPI mode */

    SELECT();                /* CS = L */
    ty = 0;
    if (send_cmd(CMD0, 0) == 1) {            /* Enter Idle state */
        Timer1 = 100;                        /* Initialization timeout of 1000 msec */
        if (send_cmd(CMD8, 0x1AA) == 1) {    /* SDC Ver2+ */
            for (n = 0; n < 4; n++) ocr[n] = rcvr_spi();
            if (ocr[2] == 0x01 && ocr[3] == 0xAA) {    /* The card can work at vdd range of 2.7-3.6V */
                do {
                    if (send_cmd(CMD55, 0) <= 1 && send_cmd(CMD41, 1UL << 30) == 0)    break;    /* ACMD41 with HCS bit */
                } while (Timer1);
                if (Timer1 && send_cmd(CMD58, 0) == 0) {    /* Check CCS bit */
                    for (n = 0; n < 4; n++) ocr[n] = rcvr_spi();
                    ty = (ocr[0] & 0x40) ? 6 : 2;
                }
            }
        } else {                            /* SDC Ver1 or MMC */
            ty = (send_cmd(CMD55, 0) <= 1 && send_cmd(CMD41, 0) <= 1) ? 2 : 1;    /* SDC : MMC */
            do {
                if (ty == 2) {
                    if (send_cmd(CMD55, 0) <= 1 && send_cmd(CMD41, 0) == 0) break;    /* ACMD41 */
                } else {
                    if (send_cmd(CMD1, 0) == 0) break;                                /* CMD1 */
                }
            } while (Timer1);
            if (!Timer1 || send_cmd(CMD16, 512) != 0)    /* Select R/W block length */
                ty = 0;
        }
    }
    CardType = ty;
    DESELECT();            /* CS = H */
    rcvr_spi();            /* Idle (Release DO) */

    if (ty) {            /* Initialization succeded */
        Stat &= ~STA_NOINIT;        /* Clear STA_NOINIT */
        set_max_speed();
    } else {            /* Initialization failed */
        power_off();
    }

    return Stat;
}



/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
    BYTE drv        /* Physical drive nmuber (0) */
)
{
    if (drv) return STA_NOINIT;        /* Supports only single drive */
    return Stat;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
    BYTE drv,            /* Physical drive nmuber (0) */
    BYTE *buff,            /* Pointer to the data buffer to store read data */
    DWORD sector,        /* Start sector number (LBA) */
    BYTE count            /* Sector count (1..255) */
)
{
    if (drv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    if (!(CardType & 4)) sector *= 512;    /* Convert to byte address if needed */

    SELECT();            /* CS = L */

    if (count == 1) {    /* Single block read */
        if ((send_cmd(CMD17, sector) == 0)    /* READ_SINGLE_BLOCK */
            && 
            #if MODE == MODE_POLLING | MODE == MODE_INTERRUPT
            rcvr_datablock(buff, 512)
            #elif MODE == MODE_INTERRUPT_BLOCK | MODE == MODE_DMA
            rcvr_largedatablock(buff)
            #endif
        )
            count = 0;
    }
    else {                /* Multiple block read */
        if (send_cmd(CMD18, sector) == 0) {    /* READ_MULTIPLE_BLOCK */
            do {
                if (!
                    #if MODE == MODE_POLLING | MODE == MODE_INTERRUPT
                    rcvr_datablock(buff, 512)
                    #elif MODE == MODE_INTERRUPT_BLOCK | MODE == MODE_DMA
                    rcvr_largedatablock(buff)
                    #endif
                ) break;
                buff += 512;
            } while (--count);
            send_cmd12();                /* STOP_TRANSMISSION */
        }
    }

    DESELECT();            /* CS = H */
    rcvr_spi();            /* Idle (Release DO) */

    return count ? RES_ERROR : RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _READONLY == 0
DRESULT disk_write (
    BYTE drv,            /* Physical drive nmuber (0) */
    const BYTE *buff,    /* Pointer to the data to be written */
    DWORD sector,        /* Start sector number (LBA) */
    BYTE count            /* Sector count (1..255) */
)
{
    if (drv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;
    if (Stat & STA_PROTECT) return RES_WRPRT;

    if (!(CardType & 4)) sector *= 512;    /* Convert to byte address if needed */

    SELECT();            /* CS = L */

    if (count == 1) {    /* Single block write */
        if ((send_cmd(CMD24, sector) == 0)    /* WRITE_BLOCK */
            && 
            #if MODE == MODE_POLLING | MODE == MODE_INTERRUPT
            xmit_datablock(buff, 0xFE)
            #elif MODE == MODE_INTERRUPT_BLOCK | MODE == MODE_DMA
            xmit_largedatablock(buff, 0xFE)
            #endif
            )
            count = 0;
    }
    
            
            
            
            
            
    else {                /* Multiple block write */
        if (CardType & 2) {
            send_cmd(CMD55, 0); send_cmd(CMD23, count);    /* ACMD23 */
        }
        if (send_cmd(CMD25, sector) == 0) {    /* WRITE_MULTIPLE_BLOCK */
            do {
                if (!
                #if MODE == MODE_POLLING | MODE == MODE_INTERRUPT
                xmit_datablock(buff, 0xFC)
                #elif MODE == MODE_INTERRUPT_BLOCK | MODE == MODE_DMA
                xmit_largedatablock(buff, 0xFC)
                #endif
                ) break;
                buff += 512;
            } while (--count);
            if (!
            #if MODE == MODE_POLLING | MODE == MODE_INTERRUPT
            xmit_datablock(0, 0xFD)
            #elif MODE == MODE_INTERRUPT_BLOCK | MODE == MODE_DMA
            xmit_largedatablock(0, 0xFD)
            #endif
            )    /* STOP_TRAN token */
                count = 1;
        }
    }

    DESELECT();            /* CS = H */
    rcvr_spi();            /* Idle (Release DO) */

    return count ? RES_ERROR : RES_OK;
}
#endif /* _READONLY */



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
    BYTE drv,        /* Physical drive nmuber (0) */
    BYTE ctrl,        /* Control code */
    void *buff        /* Buffer to send/receive control data */
)
{
    DRESULT res;
    BYTE n, csd[16], *ptr = buff;
    WORD csize;


    if (drv) return RES_PARERR;

    res = RES_ERROR;

    if (ctrl == CTRL_POWER) {
        switch (*ptr) {
        case 0:        /* Sub control code == 0 (POWER_OFF) */
            if (chk_power())
                power_off();        /* Power off */
            res = RES_OK;
            break;
        case 1:        /* Sub control code == 1 (POWER_ON) */
            power_on();                /* Power on */
            res = RES_OK;
            break;
        case 2:        /* Sub control code == 2 (POWER_GET) */
            *(ptr+1) = (BYTE)chk_power();
            res = RES_OK;
            break;
        default :
            res = RES_PARERR;
        }
    }
    else {
        if (Stat & STA_NOINIT) return RES_NOTRDY;

        SELECT();        /* CS = L */

        switch (ctrl) {
        case GET_SECTOR_COUNT :    /* Get number of sectors on the disk (DWORD) */
            if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
                if ((csd[0] >> 6) == 1) {    /* SDC ver 2.00 */
                    csize = csd[9] + ((WORD)csd[8] << 8) + 1;
                    *(DWORD*)buff = (DWORD)csize << 10;
                } else {                    /* MMC or SDC ver 1.XX */
                    n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                    csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
                    *(DWORD*)buff = (DWORD)csize << (n - 9);
                }
                res = RES_OK;
            }
            break;

        case GET_SECTOR_SIZE :    /* Get sectors on the disk (WORD) */
            *(WORD*)buff = 512;
            res = RES_OK;
            break;

        case CTRL_SYNC :    /* Make sure that data has been written */
            if (wait_ready() == 0xFF)
                res = RES_OK;
            break;

        case MMC_GET_CSD :    /* Receive CSD as a data block (16 bytes) */
            if (send_cmd(CMD9, 0) == 0        /* READ_CSD */
                && rcvr_datablock(ptr, 16))
                res = RES_OK;
            break;

        case MMC_GET_CID :    /* Receive CID as a data block (16 bytes) */
            if (send_cmd(CMD10, 0) == 0        /* READ_CID */
                && rcvr_datablock(ptr, 16))
                res = RES_OK;
            break;

        case MMC_GET_OCR :    /* Receive OCR as an R3 resp (4 bytes) */
            if (send_cmd(CMD58, 0) == 0) {    /* READ_OCR */
                for (n = 0; n < 4; n++)
                    *ptr++ = rcvr_spi();
                res = RES_OK;
            }

//        case MMC_GET_TYPE :    /* Get card type flags (1 byte) */
//            *ptr = CardType;
//            res = RES_OK;
//            break;

        default:
            res = RES_PARERR;
        }

        DESELECT();            /* CS = H */
        rcvr_spi();            /* Idle (Release DO) */
    }

    return res;
}



/*-----------------------------------------------------------------------*/
/* Device Timer Interrupt Procedure  (Platform dependent)                */
/*-----------------------------------------------------------------------*/
/* This function must be called in period of 10ms                        */

void disk_timerproc (void)
{
//    BYTE n, s;
    BYTE n;


    n = Timer1;                        /* 100Hz decrement timer */
    if (n) Timer1 = --n;
    n = Timer2;
    if (n) Timer2 = --n;

}

/*---------------------------------------------------------*/
/* User Provided Timer Function for FatFs module           */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support a real time clock.          */

DWORD get_fattime (void)
{

    return    ((2007UL-1980) << 25)    // Year = 2007
            | (6UL << 21)            // Month = June
            | (5UL << 16)            // Day = 5
            | (11U << 11)            // Hour = 11
            | (38U << 5)            // Min = 38
            | (0U >> 1)                // Sec = 0
            ;

}
