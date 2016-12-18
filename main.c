/* Description: This file contains the main function for CS7004 (EMBEDDED SYSTEM)
 * "Lightning Control System" Project. 
 *
 * Author: Manas Marawaha (MSc. Mobile and Ubiquitous Computing)
 *         marawahm@tcd.ie
 *
 * Platform: FREE RTOS
*/

/* Standard includes. */
#include <stdlib.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "console.h"
#include "semphr.h"

/* Application includes */
#include "lcd.h"
#include "led_controller.h"
#include "lcd_hw.h"
#include "lcd_grph.h"
#include "utility.h"

extern void vLCD_ISREntry(void);

/* Hardware Initialization */
static void prvSetupHardware(void);

/* mutex for button area map mutual exclusion */
SemaphoreHandle_t ButtonLock;

/* mutex for I2C mutual exclusion */
SemaphoreHandle_t BusLock;

int main (void)
{
    xQueueHandle xCmdQ;

    /* Setup the hardware for use with the Keil demo board. */
    prvSetupHardware();

    xCmdQ = xQueueCreate(MAX_EVENTS, sizeof(Command_t));

    /* Mutex for mutually exclusive access of button map 
       among multiple task */
    ButtonLock = xSemaphoreCreateMutex();

    /* Mutex for mutually exclusive access of slider map 
       among multiple task */
    BusLock = xSemaphoreCreateMutex();

    /* Start the console task */
    vStartConsole(2, BAUD_RATE);

    /* Start the UI task */
    vStartLcd(2, xCmdQ, ButtonLock);

    /* Start the LED Controller task */
    vStartSensors(2, xCmdQ, BusLock, ButtonLock);

    /* Start PIR Detection task */
    vStartPolling(2, xCmdQ, BusLock, ButtonLock);

    /* Start the FreeRTOS Scheduler...*/ 
    vTaskStartScheduler();

    while(1);
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required. */ 
	
    /* Enable UART0. */
    PCONP   |= (1 << 3);                    /* Enable UART0 power */
    PINSEL0 |= 0x00000050;                  /* Enable TxD0 and RxD0 */

	/* Initialise LCD hardware */
	lcd_hw_init();

	/* Setup LCD interrupts */
	PINSEL4 |= 1 << 26;				        /* Enable P2.13 for EINT3 function */
	EXTMODE |= 8;					        /* EINT3 edge-sensitive mode */
	EXTPOLAR &= ~0x8;				        /* Falling edge mode for EINT3 */

	/* Setup VIC for LCD interrupts */
	VICIntSelect &= ~(1 << 17);		        /* Configure vector 17 (EINT3) for IRQ */
	VICVectPriority17 = 15;			        /* Set priority 15 (lowest) for vector 17 */
	VICVectAddr17 = (unsigned long)vLCD_ISREntry;   /* Set handler vector */
}
