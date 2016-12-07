/* Standard includes. */
#include <stdlib.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "console.h"

/* Application includes */
#include "lcd.h"
#include "sensors.h"
#include "lcd_hw.h"
#include "lcd_grph.h"
#include "utility.h"

extern void vLCD_ISREntry( void );

/*
 * Hardware Initialization
 */
static void prvSetupHardware( void );

int main (void)
{
	xQueueHandle xCmdQ;
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	
	xCmdQ = xQueueCreate(MAX_EVENTS, sizeof(Command));

    /* Start the console task */
	vStartConsole(1, BAUD_RATE);
	
	/* Start the Touchscreen task */
	vStartLcd(1, xCmdQ);
//test
	/* Start the Lightning task */
	vStartSensors(1, xCmdQ);

	/* Start the FreeRTOS Scheduler ... after this we're pre-emptive multitasking ...

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	while(1);
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required. */ 
	
    /* Enable UART0. */
    PCONP   |= (1 << 3);                 /* Enable UART0 power                */
    PINSEL0 |= 0x00000050;               /* Enable TxD0 and RxD0              */

	/* Initialise LCD hardware */
	lcd_hw_init();

	/* Setup LCD interrupts */
	PINSEL4 |= 1 << 26;				/* Enable P2.13 for EINT3 function */
	EXTMODE |= 8;					/* EINT3 edge-sensitive mode */
	EXTPOLAR &= ~0x8;				/* Falling edge mode for EINT3 */

	/* Setup VIC for LCD interrupts */
	VICIntSelect &= ~(1 << 17);		/* Configure vector 17 (EINT3) for IRQ */
	VICVectPriority17 = 15;			/* Set priority 15 (lowest) for vector 17 */
	VICVectAddr17 = (unsigned long)vLCD_ISREntry;
									/* Set handler vector */

}
