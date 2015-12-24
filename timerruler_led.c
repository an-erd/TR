/*
 * timerruler_led.c
 *
 * Created: 13.12.2015 12:00:00
 * Author : AKAEM
 */ 

#ifndef _TIMER_RULER_LED_H_
#define _TIMER_RULER_LED_H_

#include <avr/io.h>
#include <util/delay.h>

#include "bitoperations.h"
#include "timerruler_const.h"
#include "timerruler_led.h"

// define array for specific LEDs, [PORT][BIT] (PORTB=2, PORTD=4)
const uint8_t ledArray[number_led][2] = {
	{4, 0}, {4, 1}, {4, 2}, {4, 3},
	{4, 4}, {4, 5}, {4, 6}, {4, 7},
	{2, 1}, };

// LED Arrays following always use [STEP][{PORTD}{PORTC}]
// define array for "line", 9 steps (1 LED moves from left to right)
const uint8_t ledArrayAllSingle[10][2] = {
	{0x00, 0x00}, {0x01, 0x00}, {0x02, 0x00}, {0x04, 0x00}, {0x08, 0x00},
	{0x10, 0x00}, {0x20, 0x00}, {0x40, 0x00}, {0x80, 0x00}, {0x00, 0x02}, };

// define array for "cascade", 9 steps (LEDs fill from left to right)
const uint8_t ledArrayAllCascade_LR[10][2] = {
	{0x00, 0x00}, {0x01, 0x00}, {0x03, 0x00}, {0x07, 0x00}, {0x0F, 0x00},
	{0x1F, 0x00}, {0x3F, 0x00}, {0x7F, 0x00}, {0xFF, 0x00}, {0xFF, 0x02}, };

// define array for "reverse cascade", 9 steps (LEDs fill from right to left
const uint8_t ledArrayAllCascade_RL[10][2] = {
	{0x00, 0x00}, {0x00, 0x02}, {0x80, 0x02}, {0xC0, 0x02}, {0xE0, 0x02},
	{0xF0, 0x02}, {0xF8, 0x02}, {0xFC, 0x02}, {0xFE, 0x02}, {0xFF, 0x02}, };

// define ORANGE array for "line", X steps (1 LED moves from left to right [][0] or rev [][1]), ONLY PORTD, PD5..7
const uint8_t ledArrayOrangeSingle[4][2] = { {0x00, 0x00}, {0x20, 0x80}, {0x40, 0x40}, {0x80, 0x20}, };
				
// define ORANGE and RED array for "cascade", X steps (orange LED fill from left to right, ONLY PORTD, PD5..7 resp. PD0..4
const uint8_t ledArrayOrangeCascade_LR[4] = { 0x00, 0x20, 0x60, 0xE0, };
const uint8_t ledArrayOrangeEffect[5][2] = { {0, 0}, {0x80, 0x20}, {0xC0, 0x60}, {0x60, 0xC0}, {0x20, 0x80}, };	// left to right [][0] or rev [][1]
const uint8_t ledArrayRedCascade_LR[6] = { 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, };

// define min and max values for sGlobalStatus.config_params
const uint8_t config_params_min_max[CONFIG_NR_CYCLES+1][2] = { {1, 5}, {1, 5}, {1, 5}, {0, 5}, };
				
// define some specific LED operations
void led_set (uint8_t led_nr, uint8_t led_state)
{
	switch (ledArray[led_nr][0])
	{
		case 2: bit_write(led_state, PORTB, BIT(ledArray[led_nr][1])); break;
		case 3: bit_write(led_state, PORTC, BIT(ledArray[led_nr][1])); break;
		case 4: bit_write(led_state, PORTD, BIT(ledArray[led_nr][1])); break;
		default: ;
	}
	return;
}

void led_flip (uint8_t led_nr)
{
	switch (ledArray[led_nr][0])
	{
		case 2: bit_flip(PORTB, BIT(ledArray[led_nr][1])); break;
		case 3: bit_flip(PORTC, BIT(ledArray[led_nr][1])); break;
		case 4: bit_flip(PORTD, BIT(ledArray[led_nr][1])); break;
		default: ;
	}
	return;
}

// turn all 9 LED on/off
void led_set_all (uint8_t led_state)
{
	volatile uint8_t i;
	for (i = 0; i < number_led; i++){
		led_set(i, led_state);
	}

	return;
}

// all 9 LED effect
void led_effect_1 (void)
{
	volatile uint8_t i;
	led_set_all(OFF);
	for (i = 0; i < number_led; i++){
		led_set (i, ON);
		_delay_ms(100);
		led_set (i, OFF);
	}
	led_set_all(OFF);

	return;
}


#endif 
