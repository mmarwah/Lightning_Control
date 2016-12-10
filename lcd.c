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
    {SLI, 25, 100, 50, 125, "+", OFF, LIGHT_GRAY},      /* Slider Left + */
    {SLD, 25, 200, 50, 225, "-", OFF, LIGHT_GRAY},      /* Slider Left - */
    {SRI, 185, 100, 210, 125, "+", OFF, LIGHT_GRAY},    /* Slider Right + */
    {SRD, 185, 200, 210, 225, "-", OFF, LIGHT_GRAY},    /* Slider Right - */
};

Slider_t slider[] = {
    {LEVEL3, {{25,188,50,197},{25,175,50,185},{25,160,50,170},{25,145,50,155},{25,130,50,140}}, LIGHT_GRAY},
    {LEVEL3, {{185,188,210,197},{185,175,210,185},{185,160,210,170},{185,145,210,155},{185,130,210,140}}, LIGHT_GRAY},
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
    int i;

    lcd_line(37, 125, 37, 200, LIGHT_GRAY);
    lcd_line(197, 125, 197, 200, LIGHT_GRAY);
    for ( region = SLI; region <= SRD; region++ ) {
        lcd_fillRect(buttons[region].x0, buttons[region].y0, buttons[region].x1, buttons[region].y1, buttons[region].color);
        /* Print Region name */
        lcd_putString( buttons[region].x0 + (((buttons[region].x1 - buttons[region].x0) - (strlen(buttons[region].display) * 5)) / 2),
                buttons[region].y0+10,
                buttons[region].display);
    }
    for (i = 0; i < MAX_SLIDER; i++) {
        Slider_Level_t temp = slider[i].level;
        lcd_fillRect(slider[i].slider_pos[temp].x0, slider[i].slider_pos[temp].y0, slider[i].slider_pos[temp].x1, slider[i].slider_pos[temp].y1                     , slider[i].color);
    }
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

    for ( i = 0; i < ( MAX_BUTTON + (MAX_SLIDER * 2)) && !result; i++) {
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
    } else if (region >= WHITEBOARD && region <= SEATING ) {   /* STATE CHANGE BASED ON INDIVIDUAL REGION */
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
    } else if (region >= SLI && region <= SRD ) {   /* SLIDERS STATES */
        if (region == SLI && slider[0].level != LEVEL5)
            slider[0].level++;
        else if (region == SLD && slider[0].level != LEVEL1)
            slider[0].level--;
        if (region == SRI && slider[1].level != LEVEL5)
            slider[1].level++;
        else if (region == SRD && slider[1].level != LEVEL1)
            slider[1].level--;
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
