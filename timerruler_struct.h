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
	uint16_t		backward_cnt_high_in_a_row;
	uint8_t			temp_state_button_pressed;
	uint8_t			temp_wait_for_button_release;
	uint8_t			button_pressed;
} SButtonControl;

typedef struct
{
	uint8_t			phase;								// see defines for PHASE_CONFIG, ...
	uint8_t			config_params[NUM_CONFIG_PARAMS];
	SButtonControl	buttons[NUM_SWITCHES];				// button debounce

	// training counter
	uint8_t			current_led_step;					// current step 1-5 of training interval
	uint8_t			backward_counter_sec_to_go;			// sec backward counter to complete step
	
	// current LED mode
	uint8_t			led_steps_threshold[2][2];			// 5 red leds threshold values for ACTIVE/REST phase,  resp. (calculated)
														// ... [0] for step 1..4, [1] for step 5, and threshold[0..1][0=ACTIVE/1=REST]
	uint8_t			green_led_mode;						// slow/fast flashing or heartbeat mode (ISR)
	uint8_t			green_led_max_cycle;				// divide 1s by number of max_cycle
	uint8_t			green_led_current_cycle;			// repeat until current_cycle == max_cycle
	uint16_t		green_led_OCR1B_basis;				// set for next compare: OCR1B = current_cycle * OCR1B_basis
	
	// all led cascade effect (orange LED effect, all LED effect)
	uint16_t		effect_led_ms_counter;				// backward counter used in ORC0A to measure specific ms length
	uint8_t			effect_led_current_step;			// used and increased in OCR0A, reset in OCR1A
	uint8_t			effect_led_direction;			// gives direction if L->R (1) or R->L (0)
	uint8_t			led_effect_ongoing;					// if (effect_led_current_step), this effect will be fired
	uint8_t			orange_led_period;					// time period in seconds until next effect

	// power optimization
	uint16_t		backward_cnt_sec_to_deep_sleep;
	
} sGlobalStatus;

#endif 
