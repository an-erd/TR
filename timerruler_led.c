/*
 * timerruler_led.c
 *
 * Created: 13.12.2015 12:00:00
 * Author : AKAEM
 */ 

#include <avr/io.h>
#include <util/delay.h>

#include "bitoperations.h"
#include "timerruler_const.h"
#include "timerruler_led.h"

// define array for specific LEDs, [PORT][BIT] (PORTB=2, PORTD=4)
const uint8_t ledArray[NUM_LEDS][2] = {
	{4, 0}, {4, 1}, {4, 2}, {4, 3},
	{4, 4}, {4, 5}, {4, 6}, {4, 7},
	{2, 1}, };

// "cascade" and "2-on effect", only PORTD, PD5..7 resp. PD0..4
const uint8_t	ledArrayOrangeCascade_LR[4]	= { 0x00, 0x20, 0x60, 0xE0, };
const uint8_t	ledArrayOrangeEffect[5][2]	= { {0, 0}, {0x80, 0x20}, {0xC0, 0x60}, {0x60, 0xC0}, {0x20, 0x80}, };
const uint8_t	ledArrayRedCascade_LR[6]	= { 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, };
const uint8_t	ledArrayAllEffect[ALL_LED_EFFECT_STEPS][2] = { {0x00, 0x00}, 
	{0x80, 0x01}, {0xC0, 0x03}, {0xE0, 0x07}, {0x70, 0x0E}, {0x38, 0x1C}, {0x1C, 0x38}, {0x0E, 0x70}, {0x07, 0xE0}, {0x03, 0xC0}, {0x01, 0x80}, };
const uint8_t	ledArrayAllCenterEffect[ALL_LED_CENTER_EFFECT_STEPS] = { 0x00, 
	0x81, 0xC3, 0x66, 0x3C, 0x18, 0x18, 0x3C, 0x66, 0xC3, 0x81, 0x00, };


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
}

// turn all 9 LED on/off
void led_set_all (uint8_t led_state)
{
	if(led_state){
		PORTB |= LED_PORTB;
		PORTD |= LED_PORTD;
		} else {
		PORTB &= ~LED_PORTB;
		PORTD &= ~LED_PORTD;
	}
} 
