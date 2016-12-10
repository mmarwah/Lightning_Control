#ifndef COMMAND_H
#define COMMAND_H

#include"lcd_grph.h"

/* Queue Size */
#define MAX_EVENTS 16
#define BAUD_RATE 19200
#define MAX_BUTTON 5
#define MAX_SLIDER 2

typedef enum Region_t
{
    MASTER = 0,
    WHITEBOARD,
    DICE,
    AISLE,
    SEATING,
    SLI,			/* Slider Left + */
    SLD,      /* Slider Left - */
    SRI,      /* Slider Right + */
    SRD       /* Slider Right - */
} Region_t;

typedef struct Command 
{
	int value;
	Region_t region;
} Command;

#if 0
typedef enum Slider_Level_t
{
    LEVEL1 = 0x19,  /* 10 % Brightness */
    LEVEL2 = 0x40,  /* 25 % Brightness */
    LEVEL3 = 0x80,  /* 50 % Brightness */
    LEVEL4 = 0xC0,  /* 75 % Brightness */
    LEVEL5 = 0xFF   /* 100 % Brightness */
} Slider_Level_t;
#endif

typedef enum Slider_Level_t
{
    LEVEL1 = 0,  /* 10 % Brightness */
    LEVEL2,      /* 25 % Brightness */
    LEVEL3,      /* 50 % Brightness */
    LEVEL4,      /* 75 % Brightness */
    LEVEL5       /* 100 % Brightness */
} Slider_Level_t;

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

typedef struct Slider_map_t
{
    unsigned int x0, y0, x1, y1;
} Slider_map_t;

typedef struct Slider_t
{
    Slider_Level_t level;
    Slider_map_t slider_pos[5];
    lcd_color_t color;
} Slider_t;

/* Button Area Map */
extern Button buttons[];
extern Slider_t slider[];
#endif
