#ifndef COMMAND_H
#define COMMAND_H

#include"lcd_grph.h"

/* Queue Size */
#define MAX_EVENTS 16
#define BAUD_RATE 19200
#define MAX_BUTTON 5

typedef struct Command 
{
	int value;
	int region;
} Command;

typedef enum Region_t
{
    MASTER = 0,
    WHITEBOARD,
    DICE,
    AISLE,
    SEATING,
    SLI,
    SLD,
    SRI,
    SRD
} Region_t;

typedef enum ButtonState
{
    OFF = 0,
    ON = 1
} ButtonState;

typedef struct button
{
    Region_t region; 
    unsigned int x0, y0, x1, y1;
    char display[32];
    ButtonState state;
    lcd_color_t color;
} Button;

/* Button Area Map */
extern Button buttons[];
#endif
