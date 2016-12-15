/* Description: This file contains the functionality of LED Controller Task for 
 * CS7004 (EMBEDDED SYSTEM) "Lightning Control System" Project. 
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

/* Application includes */
#include "sensors.h"
#include "lcd.h"
#include "utility.h"

/* Maximum task stack size */
#define sensorsSTACK_SIZE			( ( unsigned portBASE_TYPE ) 256 )

/* PwmMap Contains the map for different level of slider */
unsigned char PwmMap[] = { 0x19, 0x40, 0x80, 0xC0, 0xFF };

/* The LCD task. */
static void vSensorsTask( void *pvParameters );

void vStartSensors( unsigned portBASE_TYPE uxPriority, xQueueHandle xQueue )
{
	static xQueueHandle xCmdQ;
	
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

	/* Spawn the console task . */
	xTaskCreate( vSensorsTask, ( signed char * ) "Sensors", sensorsSTACK_SIZE, &xCmdQ, uxPriority, ( xTaskHandle * ) NULL );

	printf("Sensor task started ...\r\n");
}


/* Description: Set PCA9532 Registers
 * Parameters: Param 1: Choice [0 -> LS2, 1 -> PWM0, 2 -> PWM1] 
 *             Param 2: Data to send for selected register */
void I2C_Utils(int choice, unsigned char data)
{
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
	
    if(choice == 1) {   /* Setting LEDs using LS2 register */
        /* Send control data to PCA9532 LS2 register */
        I20DAT = 0x08;
        I20CONCLR =  I2C_SI;

        /* Wait for DATA with control word to be sent */
        while (!(I20CONSET & I2C_SI));

        /* Send data to write PCA9532 LS2 register */
        I20DAT = data;
        I20CONCLR =  I2C_SI;

        /* Wait for DATA to be sent */
        while (!(I20CONSET & I2C_SI));
    } else if (choice == 2) {   /* Set PWM0 register */
        /* Send control data to select PCA9532 PWM0 register */
        I20DAT = 0x03;
        I20CONCLR =  I2C_SI;

        /* Wait for DATA with control word to be sent */
        while (!(I20CONSET & I2C_SI));

        /* Send data to write PCA9532 PWM0 register */
        I20DAT = data;
        I20CONCLR =  I2C_SI;

        /* Wait for DATA to be sent */
        while (!(I20CONSET & I2C_SI));
    } else if (choice == 3) {   /* Set PWM1 register */
        /* Send control data to select PCA9532 PWM1 register */
        I20DAT = 0x05;
        I20CONCLR =  I2C_SI;

        /* Wait for DATA with control word to be sent */
        while (!(I20CONSET & I2C_SI));

        /* Send data to write PCA9532 PWM1 register */
        I20DAT = data;
        I20CONCLR =  I2C_SI;

        /* Wait for DATA to be sent */
        while (!(I20CONSET & I2C_SI));
    }
	/* Request send NAQ and STOP */
	I20CONSET =  I2C_STO;
	I20CONCLR =  I2C_SI | I2C_AA;

	/* Wait for STOP to be sent */
	while (I20CONSET & I2C_STO);
}

/* SetLedState returns the hex values to
 * control the LEDs based on it state */
unsigned char SetLedState()
{
    unsigned char ledstate = 0x00;

    if (buttons[WHITEBOARD].state == ON)
        ledstate |= 0x02;       /* LED8 --> PWM0 */
    if (buttons[DICE].state == ON)
        ledstate |= 0x08;       /* LED9 --> PWM0 */
    if (buttons[AISLE].state == ON)
        ledstate |= 0x30;       /* LED10 --> PWM1 */
    if (buttons[SEATING].state == ON)
        ledstate |= 0xC0;       /* LED11 --> PWM1 */
    
    return ledstate;
}

/* LED Controller task function */
static portTASK_FUNCTION( vSensorsTask, pvParameters )
{
    portTickType xLastWakeTime;
    unsigned char data;
    xQueueHandle xCmdQ;
    
    /* Command to sent in Q */
    Command_t cmd;

    /* Capture Q handler passed as argument */
    xCmdQ = * ( ( xQueueHandle * ) pvParameters );

    ( void ) pvParameters;

	printf("Starting LED Controller task ...\r\n");

    /* Set Initial state of PWM0 and PWM1 */
    I2C_Utils(2, PwmMap[slider[0].level]);      /* Default 50% Brightness */
    I2C_Utils(3, PwmMap[slider[1].level]);      /* Default 50% Brightness */

	/* initial xLastWakeTime for accurate polling interval */
	xLastWakeTime = xTaskGetTickCount();

    /* Infinite loop blocks waiting for a touch screen interrupt event from
     * the queue. */
    while( 1 )
    {
        /* Get command from Q */
        xQueueReceive(xCmdQ, &cmd, portMAX_DELAY);

        /* Set LED for buttons */
        if ( cmd.region >= MASTER && cmd.region <= PRESET2 ) {
            data = SetLedState();
            /* Set PCA9532 LEDs */
            I2C_Utils(1, data);
            /* Set PWM for left slider */    
        } else if ( cmd.region >= SLI && cmd.region <= SLD ) {
            data = PwmMap[slider[0].level];
            I2C_Utils(2, data);
            /* Set PWM for right slider */
        } else if ( cmd.region >= SRI && cmd.region <= SRD ) {
            data = PwmMap[slider[1].level];
            I2C_Utils(3, data);
            /* Set LED based on PIR detection */
        } else if ( cmd.region >= SW1 && cmd.region <= SW4 ) {
            if (buttons[cmd.region-SWITCH_REGION_OFFSET].state == OFF) {
                StateCheck(cmd.region-SWITCH_REGION_OFFSET);
                data = SetLedState();
                /* Set PCA9532 LEDs */
                I2C_Utils(1, data);
                drawScreen();
            }
        }
        /* delay before next poll */
        vTaskDelayUntil( &xLastWakeTime, 20);
    }
}
