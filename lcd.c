/* Description: This file contains the functionality of UI Task for 
* CS7004 (EMBEDDED SYSTEM) "Lightning Control System" Project. 
*
* Author: Manas Marawaha (MSc. Mobile and Ubiquitous Computing)
*         marawahm@tcd.ie
*
* Platform: FREE RTOS
*/

/* Standard Includes */
#include <stdio.h>
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Application includes */
#include "lcd.h"
#include "lcd_hw.h"
#include "lcd_grph.h"

/* Maximum task stack size */
#define lcdSTACK_SIZE			( ( unsigned portBASE_TYPE ) 256 )

/* Interrupt handlers */
extern void vLCD_ISREntry( void );
void vLCD_ISRHandler( void );

/* The LCD task. */
static void vLcdTask( void *pvParameters );

/* Semaphore for ISR/task synchronisation */
xSemaphoreHandle xLcdSemphr;

/* mutex for button area map mutual exclusion */
SemaphoreHandle_t ButtonLockUI;

portTickType xLastWakeTime;


/* Button Map for each active area */
Button buttons[] = {
    {MASTER, 80, 120, 160, 200, "MASTER", OFF, NAVY},
    {WHITEBOARD, 0, 0, 80, 80, "WHITEBOARD", OFF, PURPLE},
    {DICE, 0, 240, 80, 320, "DICE", OFF, PURPLE},
    {AISLE, 160, 0, 240, 80, "AISLE", OFF, PURPLE},
    {SEATING, 160, 240, 240, 320, "SEATING", OFF, PURPLE},
    {PRESET1, 85, 0, 155, 40, "LECTURE", OFF, DARK_GRAY},
    {PRESET2, 85, 280, 155, 320, "EXAM", OFF, DARK_GRAY},
    {SLI, 25, 100, 50, 125, "+", OFF, LIGHT_GRAY},      /* Slider Left + */
    {SLD, 25, 200, 50, 225, "-", OFF, LIGHT_GRAY},      /* Slider Left - */
    {SRI, 185, 100, 210, 125, "+", OFF, LIGHT_GRAY},    /* Slider Right + */
    {SRD, 185, 200, 210, 225, "-", OFF, LIGHT_GRAY},    /* Slider Right - */
};

/* Slider Map for each slider level*/
Slider_t slider[] = {
    {LEVEL3, {{25,188,50,197},{25,175,50,185},{25,160,50,170},{25,145,50,155},{25,130,50,140}}, LIGHT_GRAY},
    {LEVEL3, {{185,188,210,197},{185,175,210,185},{185,160,210,170},{185,145,210,155},{185,130,210,140}}, LIGHT_GRAY},
};

char *SLevelMap[] = {"MIN", "25%", "50%", "75%", "MAX"};

void vStartLcd( unsigned portBASE_TYPE uxPriority, xQueueHandle xQueue, xSemaphoreHandle xButtonMutex )
{
	static xQueueHandle xCmdQ;
	xCmdQ = xQueue;
	ButtonLockUI = xButtonMutex;
    
	/* Semaphore Synchronization for TS events */
	vSemaphoreCreateBinary(xLcdSemphr);

	/* Spawn the console task. */
	xTaskCreate( vLcdTask, ( signed char * ) "Lcd", lcdSTACK_SIZE, &xCmdQ, uxPriority, ( xTaskHandle * ) NULL );
}

/* Drawing Current State of slider */
void DrawSlider()
{
    int i, x0, x1, y0, y1;
    Region_t region;

    lcd_line(37, 125, 37, 200, LIGHT_GRAY);
    lcd_line(197, 125, 197, 200, LIGHT_GRAY);

    /* Draw Active areas for both sliders */
    for ( region = SLI; region <= SRD; region++ ) {
        lcd_fillRect(buttons[region].x0, buttons[region].y0, buttons[region].x1, 
                     buttons[region].y1, buttons[region].color);
        /* Print Region name */
        lcd_putString( buttons[region].x0 + (((buttons[region].x1 - buttons[region].x0) - (strlen(buttons[region].display) * 5)) / 2),
                buttons[region].y0+10,
                buttons[region].display);
    }

    /* Draw Current state of slider */
    for (i = 0; i < MAX_SLIDER; i++) {
        Slider_Level_t temp = slider[i].level;
				x0 = slider[i].slider_pos[temp].x0;
				y0 = slider[i].slider_pos[temp].y0;
				x1 = slider[i].slider_pos[temp].x1;
				y1 = slider[i].slider_pos[temp].y1;
			
        lcd_fillRect(x0, y0, x1, y1, slider[i].color);
			  /* Print Region name */
        lcd_putString( x0 + (((x1 - x0) - 15) / 2), y0+1,
                SLevelMap[temp]);
    }
}

/* Drawing Current State of Buttons */
void drawButton(Button *button)
{
    char buffer[5];

    if(button->state == OFF)
        strcpy(buffer, "OFF");
    else
        strcpy(buffer, "ON");
				/* Enter Critical Section */
	if(xSemaphoreTake(ButtonLockUI, 1000)) {
    lcd_fillRect(button->x0, button->y0, button->x1, button->y1, button->color);

    if ( button->region == PRESET1 || button->region == PRESET2 ) {
        /* Print Region name */
        lcd_putString( button->x0 + (((button->x1 - button->x0) - (strlen(button->display) * 5)) / 2),
                button->y0 + 20,
                button->display);
    } else {
        /* Print Region name */
        lcd_putString( button->x0 + (((button->x1 - button->x0) - (strlen(button->display) * 5)) / 2),
                button->y0 + 29,
                button->display);
        /* Print Button Status */
        lcd_putString( button->x0 + (((button->x1 - button->x0) - 15) / 2),
                button->y0 + 45,
                buffer);
    }
		xSemaphoreGive(ButtonLockUI);
  }
	/* Exit Critical Section */
}

/* Drawing Screen with current scene */
void drawScreen()
{
    int i;
	
    lcd_fillScreen(WHITE);
    DrawSlider();
    for (i = 0; i < (MAX_BUTTON + MAX_PRESET); i++) {
        drawButton(&buttons[i]);
    }
}

/* Get the region of activity using x and y positions */
static Button * getButton(unsigned int x, unsigned int y)
{
    int i;
    Button *result = 0;

		/* Enter Critical Section */
		if(xSemaphoreTake(ButtonLockUI, 1000)) {
    for ( i = 0; i < ( MAX_BUTTON + (MAX_SLIDER * 2) + MAX_PRESET) && !result; i++) {
        if (x >= buttons[i].x0 && x <= buttons[i].x1
                && y >= buttons[i].y0 && y <= buttons[i].y1) {
            result = &buttons[i];
        }
    }
					xSemaphoreGive(ButtonLockUI);
  }
	/* Exit Critical Section */

    return result;
}

/* Drawing Screen with current scene */
void drawPopUp()
{
		lcd_fillRect(25, 100, 210, 225, RED);
		lcd_putString( 50, 160, "PLEASE SWITCH ON LIGHT!");
		vTaskDelayUntil( &xLastWakeTime, 500 ); 
}

/* State machine implementation */
void StateCheck(Region_t region)
{
    int i;
		/* Enter Critical Section */
	if(xSemaphoreTake(ButtonLockUI, 1000)) {
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
    } else if (region == PRESET1) {   /* PRESET-1, Lecture Mode */
        buttons[MASTER].state = OFF;
        buttons[WHITEBOARD].state = ON;
        buttons[DICE].state = ON;
        buttons[AISLE].state = ON;
        buttons[SEATING].state = OFF;
    } else if (region == PRESET2) {   /* PRESET-2, Exam Mode */
        buttons[MASTER].state = OFF;
        buttons[WHITEBOARD].state = OFF;
        buttons[DICE].state = ON;
        buttons[AISLE].state = ON;
        buttons[SEATING].state = ON;
    } else if (region >= SLI && region <= SLD ) {   /* LEFT SLIDERS STATES */
				if ((region == SLI || region == SLD) && (buttons[WHITEBOARD].state == ON || buttons[DICE].state == ON)) { 
        if (region == SLI && slider[0].level != LEVEL5)
            slider[0].level++;
        else if (region == SLD && slider[0].level != LEVEL1)
            slider[0].level--;
			} else {
				drawPopUp();
			}
		} else if (region >= SRI && region <= SRD ) {   /* RIGHT SLIDERS STATES */		
			if ((region == SRI || region == SRD) && (buttons[AISLE].state == ON || buttons[SEATING].state == ON)) { 
        if (region == SRI && slider[1].level != LEVEL5)
            slider[1].level++;
        else if (region == SRD && slider[1].level != LEVEL1)
            slider[1].level--;
			} else {
				drawPopUp();
			}
    }
			xSemaphoreGive(ButtonLockUI);
  }
	/* Exit Critical Section */
}

/* UI Task function */
static portTASK_FUNCTION( vLcdTask, pvParameters )
{
    unsigned int pressure;
    unsigned int xPos;
    unsigned int yPos;
    xQueueHandle xCmdQ;
    Button *pressedButton;
    
    /* Command to sent in Q */
    Command_t cmd;

    ( void ) pvParameters;

    /* Capture Q handler passed as argument */
    xCmdQ = * ( ( xQueueHandle * ) pvParameters );

    printf("Touchscreen task running\r\n");

    /* Initialise LCD display */
    lcd_init();

    /* Display initial scene */
    drawScreen();

    for( ;; )
    {
        EXTINT = 8;						/* Reset EINT3 */

        VICIntEnable = 1 << 17;			/* Enable interrupts on vector 17 */

        /* Block on a queue waiting for an event from the TS interrupt handler */
        xSemaphoreTake(xLcdSemphr, portMAX_DELAY);

        /* Disable TS interrupt vector (VIC) (vector 17) */
        VICIntEnClr = 1 << 17;

        /* Measure next sleep interval from this point */
        xLastWakeTime = xTaskGetTickCount();

        /* Keep polling until pressure == 0 */
        getTouch(&xPos, &yPos, &pressure);

        pressedButton = getButton(xPos, yPos);
        if (pressedButton) {
            //lcd_fillScreen(pressedButton->color);
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

/* Interrupt handler for touch screen */
void vLCD_ISRHandler( void )
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	/* indicate to the task above that an event has occurred */
	xSemaphoreGiveFromISR(xLcdSemphr, &xHigherPriorityTaskWoken);

	EXTINT = 8;					/* Reset EINT3 */
	VICVectAddr = 0;			/* Clear VIC interrupt */

	portEXIT_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
