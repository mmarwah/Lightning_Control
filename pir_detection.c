/* Description: This file contains the functionality of PIR Detection and Timmer 
 * Task for CS7004 (EMBEDDED SYSTEM) "Lightning Control System" Project. 
 *
 * Author: Manas Marawaha (MSc. Mobile and Ubiquitous Computing)
 *         marawahm@tcd.ie
 *
 * Platform: FREE RTOS
 */

/* Standard includes. */
#include <stdio.h>
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lpc24xx.h"
#include "timers.h"

/* Application includes */
#include "led_controller.h"
#include "utility.h"
#include "lcd.h"

/* Maximum task stack size */
#define pirSTACK_SIZE			( ( unsigned portBASE_TYPE ) 256 )

/* Q handle shared between Timer and PIR detection task */
static xQueueHandle xCmdQ;

/* Timer handlers */
TimerHandle_t xTimers[MAX_TIMER];

/* The LCD task. */
static void vPIRTask( void *pvParameters );

/* Command for Q */
Command_t cmd_poll, cmd_timer;

/* mutex for button area map mutual exclusion */
SemaphoreHandle_t ButtonLockPoll;

/* mutex for I2C mutual exclusion */
SemaphoreHandle_t BusLockPoll;

/* Timer Callback function */
void vTimerCallback( TimerHandle_t xExpiredTimer )
{
    Region_t TimmerID = ( Region_t ) pvTimerGetTimerID( xExpiredTimer );

    if (buttons[TimmerID + 1].state == ON) {
        StateCheck(TimmerID + 1);
        /* Send event to LED controller task to turn OFF 
           region related to expired timer */
        cmd_timer.region = TimmerID + 1;
        xQueueSendToBack(xCmdQ, &cmd_timer, portMAX_DELAY);
        drawScreen();
    }
}

void vStartPolling( unsigned portBASE_TYPE uxPriority, xQueueHandle xQueue, xSemaphoreHandle xBusMutex, xSemaphoreHandle xButtonMutex)
{
    int count;
    xCmdQ = xQueue;
    ButtonLockPoll = xButtonMutex;
    BusLockPoll = xBusMutex;

    PCONP    |=  (1 << 7);                /* Enable power for I2C0 */

    /* Initialize pins for SDA (P0.27) and SCL (P0.28) functions */
    PINSEL1  &= ~0x03C00000;
    PINSEL1  |=  0x01400000;

    /* Clear I2C state machine */
    I20CONCLR =  I2C_AA | I2C_SI | I2C_STA | I2C_I2EN;

    /* Setup I2C clock speed */
    I20SCLL   =  0x80;
    I20SCLH   =  0x80;

    I20CONSET =  I2C_I2EN;

    /* Create Timers */
    for ( count = 0; count < MAX_TIMER; count++ ) {
        xTimers[ count ] = xTimerCreate ( "Timer", pdMS_TO_TICKS( 5000 ), pdFALSE,
                ( void * ) count, vTimerCallback );
        if ( xTimers[ count ] == NULL ) 
            printf("Timer %d creation failed\n", count);
    }

    /* Spawn the console task . */
    xTaskCreate( vPIRTask, ( signed char * ) "PIR", pirSTACK_SIZE, &xCmdQ, uxPriority, ( xTaskHandle * ) NULL );

    printf("PIR Poll task started ...\r\n");
}

/* Get I2C button status */
unsigned char getButtons()
{
    unsigned char ledData;
    /* Enter Critical Section */
    if(xSemaphoreTake(BusLockPoll, 1000)) {

        /* Initialise */
        I20CONCLR =  I2C_AA | I2C_SI | I2C_STA | I2C_STO;

        /* Request send START */
        I20CONSET =  I2C_STA;

        /* Wait for START to be sent */
        while (!(I20CONSET & I2C_SI));

        /* Request send PCA9532 ADDRESS and R/W bit and clear SI */		
        I20DAT    =  0xC0;
        I20CONCLR =  I2C_SI | I2C_STA;

        /* Wait for ADDRESS and R/W to be sent */
        while (!(I20CONSET & I2C_SI));

        /* Send control word to read PCA9532 INPUT0 register */
        I20DAT = 0x00;
        I20CONCLR =  I2C_SI;

        /* Wait for DATA with control word to be sent */
        while (!(I20CONSET & I2C_SI));

        /* Request send repeated START */
        I20CONSET =  I2C_STA;
        I20CONCLR =  I2C_SI;

        /* Wait for START to be sent */
        while (!(I20CONSET & I2C_SI));

        /* Request send PCA9532 ADDRESS and R/W bit and clear SI */		
        I20DAT    =  0xC1;
        I20CONCLR =  I2C_SI | I2C_STA;

        /* Wait for ADDRESS and R/W to be sent */
        while (!(I20CONSET & I2C_SI));

        I20CONCLR = I2C_SI;

        /* Wait for DATA to be received */
        while (!(I20CONSET & I2C_SI));

        ledData = I20DAT;

        /* Request send NAQ and STOP */
        I20CONSET =  I2C_STO;
        I20CONCLR =  I2C_SI | I2C_AA;

        /* Wait for STOP to be sent */
        while (I20CONSET & I2C_STO);
        xSemaphoreGive(BusLockPoll);
    }
    /* Exit Critical Section */

    return ledData ^ 0xf;
}

/* Task function for PIR detection */
static portTASK_FUNCTION( vPIRTask, pvParameters )
{
    portTickType xLastWakeTime;
    unsigned char buttonState;
    unsigned int i;
    unsigned char mask;
    xQueueHandle xCmdQ;    

    /* Capture Q handler passed as argument */
    xCmdQ = * ( ( xQueueHandle * ) pvParameters );

    /* Just to stop compiler warnings. */
    ( void ) pvParameters;

    printf("Starting switch poll ...\r\n");

    /* initial xLastWakeTime for accurate polling interval */
    xLastWakeTime = xTaskGetTickCount();

    /* Infinite loop blocks waiting for a touch screen interrupt event from
     * the queue. */
    while( 1 )
    {
        /* Read buttons */
        buttonState = getButtons();

        /* iterate over each of the 4 LS bits looking for changes in state */
        for (i = 0; i <= 3; i++)
        {
            mask = 1 << i;

            if ((buttonState & mask))
            {
                /* Send event to LED controller task with PIR ID */
                cmd_poll.region = (SW1+i);
                xQueueSendToBack(xCmdQ, &cmd_poll, portMAX_DELAY);
                /* Start region based timer */
                xTimerStart( xTimers[i], 0 );
            }
        } 
        /* delay before next poll */
        vTaskDelayUntil( &xLastWakeTime, 200);
    }
}
