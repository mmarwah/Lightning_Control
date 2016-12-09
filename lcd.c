/* 
	Task that initialises the EA QVGA LCD display
	with touch screen controller and processes touch screen
	interrupt events.
*/

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "lcd.h"
#include "lcd_hw.h"
#include "lcd_grph.h"
#include <stdio.h>
#include <string.h>
#include "utility.h"

/* Maximum task stack size */
#define lcdSTACK_SIZE			( ( unsigned portBASE_TYPE ) 256 )

/* Interrupt handlers */
extern void vLCD_ISREntry( void );
void vLCD_ISRHandler( void );

/* The LCD task. */
static void vLcdTask( void *pvParameters );

/* Semaphore for ISR/task synchronisation */
xSemaphoreHandle xLcdSemphr;

Command cmd;

Button buttons[] = {
    {MASTER, 80, 120, 160, 200, "MASTER", OFF, BLUE},
    {WHITEBOARD, 0, 0, 80, 80, "WHITEBOARD", OFF, OLIVE},
    {DICE, 0, 240, 80, 320, "DICE", OFF, CYAN},
    {AISLE, 160, 0, 240, 80, "AISLE", OFF, YELLOW},
    {SEATING, 160, 240, 240, 320, "SEATING", OFF, RED},
    {SLI, 25, 100, 50, 125, "+", OFF, LIGHT_GRAY},
    {SLD, 25, 200, 50, 225, "-", OFF, LIGHT_GRAY},
    {SRI, 185, 100, 210, 125, "+", OFF, LIGHT_GRAY},
    {SRD, 185, 200, 210, 225, "-", OFF, LIGHT_GRAY},
};

void vStartLcd( unsigned portBASE_TYPE uxPriority, xQueueHandle xQueue )
{
	static xQueueHandle xCmdQ;
	
	xCmdQ = xQueue;
	
	vSemaphoreCreateBinary(xLcdSemphr);

	/* Spawn the console task. */
	xTaskCreate( vLcdTask, ( signed char * ) "Lcd", lcdSTACK_SIZE, &xCmdQ, uxPriority, ( xTaskHandle * ) NULL );
}

void DrawSlider()
{
    Region_t region;

    lcd_line(37, 125, 37, 200, LIGHT_GRAY);
    lcd_line(197, 125, 197, 200, LIGHT_GRAY);
    for ( region = SLI; region <= SRD; region++ ) {
        lcd_fillRect(buttons[region].x0, buttons[region].y0, buttons[region].x1, buttons[region].y1, buttons[region].color);
        /* Print Region name */
        lcd_putString( buttons[region].x0 + (((buttons[region].x1 - buttons[region].x0) - (strlen(buttons[region].display) * 5)) / 2),
                buttons[region].y0+10,
                buttons[region].display);
    }
    //lcd_fillRect(185, 145, 210, 155, LIGHT_GRAY);
    //lcd_fillRect(25, 145, 50, 155, LIGHT_GRAY);
}

static void drawButton(Button *button)
{
    char buffer[5];

    if(button->state == OFF)
        strcpy(buffer, "OFF");
    else
        strcpy(buffer, "ON");

    lcd_fillRect(button->x0, button->y0, button->x1, button->y1, button->color);

    /* Print Region name */
    lcd_putString( button->x0 + (((button->x1 - button->x0) - (strlen(button->display) * 5)) / 2),
            button->y0 + 29,
            button->display);

    /* Print Button Status */
    lcd_putString( button->x0 + (((button->x1 - button->x0) - 15) / 2),
            button->y0 + 45,
            buffer);
}

static void drawScreen()
{
    int i;
    lcd_fillScreen(WHITE);
    DrawSlider();

    for (i = 0; i < MAX_BUTTON; i++) {
        drawButton(&buttons[i]);
    }
}

static Button * getButton(unsigned int x, unsigned int y)
{
    int i;
    Button *result = 0;

    for ( i = 0; i < MAX_BUTTON && !result; i++) {
        if (x >= buttons[i].x0 && x <= buttons[i].x1
                && y >= buttons[i].y0 && y <= buttons[i].y1) {
            result = &buttons[i];
        }
    }

    return result;
}

void StateCheck(Region_t region)
{
	int i;
    if (region == MASTER) {     /* STATE CHANGE BASED ON MASTER */
        if (buttons[WHITEBOARD].state == ON && buttons[DICE].state == ON 
                && buttons[AISLE].state == ON && buttons[SEATING].state == ON) {
            for ( i = 0; i < MAX_BUTTON; i++ ) {
                buttons[i].state = OFF;
            } 
        } else {
            for ( i = 0; i < MAX_BUTTON; i++ ) {
                buttons[i].state = ON;
            }
        }
    } else {    /* STATE CHANGE BASED ON SINGLE REGION */
        if (buttons[region].state == ON) {
            buttons[region].state = OFF;
            buttons[MASTER].state = OFF;
        } else if (buttons[region].state == OFF) {
            buttons[region].state = ON;
            if (buttons[WHITEBOARD].state == ON && buttons[DICE].state == ON 
                    && buttons[AISLE].state == ON && buttons[SEATING].state == ON) {
                buttons[MASTER].state = ON;
            }
        }
    }
}

static portTASK_FUNCTION( vLcdTask, pvParameters )
{
    unsigned int pressure;
    unsigned int xPos;
    unsigned int yPos;
    portTickType xLastWakeTime;
    xQueueHandle xCmdQ;
    Button *pressedButton;

    /* Just to stop compiler warnings. */
    ( void ) pvParameters;

    xCmdQ = * ( ( xQueueHandle * ) pvParameters );

    printf("Touchscreen task running\r\n");

    /* Initialise LCD display */
    lcd_init();

    drawScreen();

    /* Infinite loop blocks waiting for a touch screen interrupt event from
     * the queue. */
    for( ;; )
    {
        /* Clear TS interrupts (EINT3) */
        /* Reset and (re-)enable TS interrupts on EINT3 */
        EXTINT = 8;						/* Reset EINT3 */

        /* Enable TS interrupt vector (VIC) (vector 17) */
        VICIntEnable = 1 << 17;			/* Enable interrupts on vector 17 */

        /* Block on a queue waiting for an event from the TS interrupt handler */
        xSemaphoreTake(xLcdSemphr, portMAX_DELAY);

        /* Disable TS interrupt vector (VIC) (vector 17) */
        VICIntEnClr = 1 << 17;

        /* Measure next sleep interval from this point */
        xLastWakeTime = xTaskGetTickCount();

        /* Start polling the touchscreen pressure and position ( getTouch(...) ) */
        /* Keep polling until pressure == 0 */
        getTouch(&xPos, &yPos, &pressure);

        pressedButton = getButton(xPos, yPos);
        if (pressedButton) {
            lcd_fillScreen(pressedButton->color);
            cmd.region = pressedButton->region;
            StateCheck(pressedButton->region);

            /* Inform LED task about the region pressed */
            xQueueSendToBack(xCmdQ, &cmd, portMAX_DELAY);
        }

        while (pressure > 0)
        {
            /* Get current pressure */
            getTouch(&xPos, &yPos, &pressure);

            /* Delay to give us a 25ms periodic TS pressure sample */
            vTaskDelayUntil( &xLastWakeTime, 25 );
        }

        drawScreen();
    }
}


void vLCD_ISRHandler( void )
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	/* Process the touchscreen interrupt */
	/* We would want to indicate to the task above that an event has occurred */
	xSemaphoreGiveFromISR(xLcdSemphr, &xHigherPriorityTaskWoken);

	EXTINT = 8;					/* Reset EINT3 */
	VICVectAddr = 0;			/* Clear VIC interrupt */

	/* Exit the ISR.  If a task was woken by either a character being received
	or transmitted then a context switch will occur. */
	portEXIT_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
