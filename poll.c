/* 
	Sample task that initialises the EA QVGA LCD display
	with touch screen controller and processes touch screen
	interrupt events.

*/

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lpc24xx.h"
#include <stdio.h>
#include <string.h>
#include "sensors.h"
#include "utility.h"
#include "timers.h"
#include "lcd.h"

#define I2C_AA      0x00000004
#define I2C_SI      0x00000008
#define I2C_STO     0x00000010
#define I2C_STA     0x00000020
#define I2C_I2EN    0x00000040

/* Maximum task stack size */
#define sensorsSTACK_SIZE			( ( unsigned portBASE_TYPE ) 256 )

static xQueueHandle xCmdQ;
TimerHandle_t xTimers[MAX_TIMER];

/* The LCD task. */
static void vSensorsTask( void *pvParameters );
Command cmd_poll, cmd_timer;


void vTimerCallback( TimerHandle_t xExpiredTimer )
{
   Region_t TimmerID = ( Region_t ) pvTimerGetTimerID( xExpiredTimer );
	
	 printf("From Timer Callback %d\n", TimmerID);
	
   if (buttons[TimmerID + 1].state == ON) {
       StateCheck(TimmerID + 1);
       cmd_timer.region = TimmerID + 1;
       xQueueSendToBack(xCmdQ, &cmd_timer, portMAX_DELAY);
       drawScreen();
   }
}

void vStartPolling( unsigned portBASE_TYPE uxPriority, xQueueHandle xQueue )
{
    int count;
    xCmdQ = xQueue;

	/* Enable and configure I2C0 */
	PCONP    |=  (1 << 7);                /* Enable power for I2C0              */

	/* Initialize pins for SDA (P0.27) and SCL (P0.28) functions                */
	PINSEL1  &= ~0x03C00000;
	PINSEL1  |=  0x01400000;

	/* Clear I2C state machine                                                  */
	I20CONCLR =  I2C_AA | I2C_SI | I2C_STA | I2C_I2EN;
	
	/* Setup I2C clock speed                                                    */
	I20SCLL   =  0x80;
	I20SCLH   =  0x80;
	
	I20CONSET =  I2C_I2EN;

    /* Create Timer */
    for ( count = 0; count < MAX_TIMER; count++ ) {
        xTimers[ count ] = xTimerCreate ( "Timer", pdMS_TO_TICKS( 5000 ), pdFALSE,
                                          ( void * ) count, vTimerCallback );
        if ( xTimers[ count ] == NULL ) 
            printf("Timer creation failed\n");
    }

	/* Spawn the console task . */
	xTaskCreate( vSensorsTask, ( signed char * ) "Poll", sensorsSTACK_SIZE, &xCmdQ, uxPriority, ( xTaskHandle * ) NULL );

	printf("Poll task started ...\r\n");
}

/* Get I2C button status */
unsigned char getButtons()
{
	unsigned char ledData;

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

	//printf("LED data %u ON\r\n", ledData);
	return ledData ^ 0xf;
}

static portTASK_FUNCTION( vSensorsTask, pvParameters )
{
	portTickType xLastWakeTime;
	unsigned char buttonState;
	unsigned int i;
	unsigned char mask;
	xQueueHandle xCmdQ;    

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
								printf("Button %u is ON\r\n", i);
                cmd_poll.region = (SW1+i);
                xQueueSendToBack(xCmdQ, &cmd_poll, portMAX_DELAY);
                /* Start region based timer */
								printf("Starting Timer %d\n", i);
                xTimerStart( xTimers[i], 0 );
            }
        } 
        /* delay before next poll */
    	vTaskDelayUntil( &xLastWakeTime, 200);
	}
}
