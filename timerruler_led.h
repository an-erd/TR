/*
 * timerruler_led.h
 *
 * Created: 13.12.2015 12:00:00
 * Author : AKAEM
 */ 

#ifndef _TIMER_RULER_LED_H_
#define _TIMER_RULER_LED_H_

#include <avr/io.h>
#include "bitoperations.h"
#include "timerruler_const.h"

// define array for specific LEDs, [PORT][BIT] (PORTB=2, PORTD=4)
extern const uint8_t ledArray[number_led][2];

// LED Arrays following always use [STEP][{PORTD}{PORTC}]
// define array for "line", 9 steps (1 LED moves from left to right)
extern const uint8_t ledArrayAllSingle[10][2];

// define array for "cascade", 9 steps (LEDs fill from left to right)
extern const uint8_t ledArrayAllCascade_LR[10][2];

// define array for "reverse cascade", 9 steps (LEDs fill from right to left
extern const uint8_t ledArrayAllCascade_RL[10][2];

// define ORANGE array for "line", X steps (1 LED moves from left to right [][0] or rev [][1]), ONLY PORTD, PD5..7
extern const uint8_t ledArrayOrangeSingle[4][2];
				
// define ORANGE and RED array for "cascade", X steps (orange LED fill from left to right, ONLY PORTD, PD5..7 resp. PD0..4
extern const uint8_t ledArrayOrangeCascade_LR[4];
extern const uint8_t ledArrayOrangeEffect[5][2];	// left to right [][0] or rev [][1]
extern const uint8_t ledArrayRedCascade_LR[6];

// define min and max values for sGlobalStatus.config_params
extern const uint8_t config_params_min_max[CONFIG_NR_CYCLES+1][2];
				
// define some specific LED operations

// set a specific led to given state
void led_set (uint8_t led_nr, uint8_t led_state);

// just flip the led ON->OFF or OFF->ON
void led_flip (uint8_t led_nr);

// turn all 9 LED on/off
void led_set_all (uint8_t led_state);

// all 9 LED effect
void led_effect_1 (void);

#endif 
