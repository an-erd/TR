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

extern const uint8_t	ledArray[NUM_LEDS][2];						// define array for addressing LEDs, [PORT][BIT] (PORTB=2, PORTD=4)

// "cascade" and "2-on effect", only PORTD, PD5..7 resp. PD0..4
extern const uint8_t	ledArrayOrangeCascade_LR[4];				// fill from L->R
extern const uint8_t	ledArrayOrangeEffect[5][2];					// effect "2 on" from L->R [][0] or rev [][1]
extern const uint8_t	ledArrayRedCascade_LR[6];					// fill from L->R
extern const uint8_t	ledArrayAllEffect[ALL_LED_EFFECT_STEPS][2];	// effect from L->R or rev for all PORTD (red/orange leds)
extern const uint8_t	ledArrayAllCenterEffect[ALL_LED_CENTER_EFFECT_STEPS]; // to center and back

// set a specific led to given state
void led_set (uint8_t led_nr, uint8_t led_state);

// just flip the led ON->OFF or OFF->ON
void led_flip (uint8_t led_nr);

// turn all 9 LED on/off
void led_set_all (uint8_t led_state);


#endif 
