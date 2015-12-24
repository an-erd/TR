/*
 * timerruler_struct.h
 *
 * Created: 13.12.2015 12:00:00
 * Author : AKAEM
 */ 

#ifndef _TIMER_RULER_STRUCT_H_
#define _TIMER_RULER_STRUCT_H_

#include "timerruler_const.h"

typedef struct
{
	uint16_t counter_high_in_a_row;
	uint8_t temp_state_button_pressed;
	uint8_t temp_wait_for_button_release;
	uint8_t button_pressed;
} SButtonControl;

typedef struct
{
	uint8_t			phase;								// see defines for PHASE_CONFIG, ...
	
	uint8_t			config_params[NUM_CONFIG_PARAMS];	// hold the configuration params
	uint8_t			interval_basis_sec;					// x sec / interval (calculated)
	uint8_t			led_steps_threshold[5][2];			// 5 red leds threshold values for ACTIVE/REST phase, resp. (calculated),
												// ...threshold[step 0..4][0=ACTIVE/1=REST]
	uint8_t			current_led_step;					// current step 1-5 of training interval
	uint8_t			backward_counter_sec_to_go;			// sec backward counter to complete step
	
	SButtonControl	buttons[NUM_SWITCHES];
	
	// current LED mode
	uint8_t			green_led_mode;						// slow/fast flashing or heartbeat mode (ISR)
	uint8_t			green_led_max_cycle;				// divide 1s by number of max_cycle
	uint8_t			green_led_current_cycle;			// repeat until current_cycle == max_cycle
	uint16_t		green_led_OCR1B_basis;				// set for next compare: OCR1B = current_cycle * OCR1B_basis
	
	// orange LED effect
	uint16_t		orange_led_counter;				// counter used in ORC0A to measure specific ms length
	uint8_t			orange_led_current_step;			// used and increased in OCR0A, reset in OCR1A
	uint8_t			orange_led_max_step;
	uint8_t			orange_led_period;					// time period in seconds until next effect

	uint8_t			PINChistory;
} sGlobalStatus;

#endif 
