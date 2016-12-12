/* 
	Sample task that initialises the EA QVGA LCD display
	with touch screen controller and processes touch screen
	interrupt events.

	Jonathan Dukes (jdukes@scss.tcd.ie)
*/

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lpc24xx.h"
#include <stdio.h>
#include <string.h>
#include "sensors.h"
#include "utility.h"

#define I2C_AA      0x00000004
#define I2C_SI      0x00000008
#define I2C_STO     0x00000010
#define I2C_STA     0x00000020
#define I2C_I2EN    0x00000040


/* Maximum task stack size */
#define sensorsSTACK_SIZE			( ( unsigned portBASE_TYPE ) 256 )
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


/* Set PCA9532 LEDs */
void I2C_Utils(int choice, unsigned char *LedMap, unsigned char pwm)
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
	
    if(choice == 1) {   /* Setting LEDs using LS2 register */
        /* Send control data to PCA9532 LS2 register */
        I20DAT = 0x08;
        I20CONCLR =  I2C_SI;

        /* Wait for DATA with control word to be sent */
        while (!(I20CONSET & I2C_SI));

        /* Update LED States */
        I20DAT = ( *LedMap );
        I20CONCLR =  I2C_SI;

        /* Wait for DATA to be sent */
        while (!(I20CONSET & I2C_SI));
    } else if (choice == 2) { /* Getting current state of LEDs from INPUT 1 register */
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
        ( *LedMap ) = ledData ^ 0xf;
    } else if (choice == 3) {   /* Set PWM0 register */
        /* Send control data to select PCA9532 PWM0 register */
        I20DAT = 0x03;
        I20CONCLR =  I2C_SI;

        /* Wait for DATA with control word to be sent */
        while (!(I20CONSET & I2C_SI));

        /* Send data to write PCA9532 PWM0 register */
        I20DAT = pwm;
        I20CONCLR =  I2C_SI;

        /* Wait for DATA to be sent */
        while (!(I20CONSET & I2C_SI));
    } else if (choice == 4) {   /* Set PWM1 register */
        /* Send control data to select PCA9532 PWM1 register */
        I20DAT = 0x05;
        I20CONCLR =  I2C_SI;

        /* Wait for DATA with control word to be sent */
        while (!(I20CONSET & I2C_SI));

        /* Send data to write PCA9532 PWM1 register */
        I20DAT = pwm;
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

static portTASK_FUNCTION( vSensorsTask, pvParameters )
{
    portTickType xLastWakeTime;
    unsigned char data;
    unsigned char PWM0 = 0x80;      /* Default 50% Brightness */
    unsigned char PWM1 = 0x80;      /* Default 50% Brightness */
    xQueueHandle xCmdQ;
    Command cmd;

    xCmdQ = * ( ( xQueueHandle * ) pvParameters );

    /* Just to stop compiler warnings. */
    ( void ) pvParameters;

	printf("Starting sensor poll ...\r\n");

    /* Set PWM0 and PWM1 */
    I2C_Utils(3, &data, PWM0);
    I2C_Utils(4, &data, PWM1);

	/* initial xLastWakeTime for accurate polling interval */
	xLastWakeTime = xTaskGetTickCount();

    /* Infinite loop blocks waiting for a touch screen interrupt event from
     * the queue. */
    while( 1 )
    {
        /* Get command from Q */
        xQueueReceive(xCmdQ, &cmd, portMAX_DELAY);

        if ( cmd.region >= MASTER && cmd.region <= PRESET2 ) {
            data = SetLedState();
            /* Set PCA9532 LEDs */
            I2C_Utils(1, &data, PWM0);
        } else if ( cmd.region >= SLI && cmd.region <= SLD ) {
            PWM0 = PwmMap[slider[0].level];
            I2C_Utils(3, &data, PWM0);
        } else if ( cmd.region >= SRI && cmd.region <= SRD ) {
            PWM1 = PwmMap[slider[1].level];
            I2C_Utils(4, &data, PWM1);
        } else if ( cmd.region >= SW1 && cmd.region <= SW4 ) {
            printf("INPUT SWITCH PRESSED \r\n");
        }

        /* delay before next poll */
        vTaskDelayUntil( &xLastWakeTime, 20);
    }
}
