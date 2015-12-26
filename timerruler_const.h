/*
 * timerruler_const.h
 *
 * Created: 13.12.2015 12:00:00
 * Author : AKAEM
 */ 

#ifndef _TIMER_RULER_CONS_H_
#define _TIMER_RULER_CONS_H_

#define NUM_LEDS						9
#define NUM_INTERVAL_LEDS				5
#define INTERVAL_LENGTH					15

// power saving mode wait this time in seconds go go to deep sleep
#define DEEP_SLEEP_SEC_MAINPROG			15
#define DEEP_SLEEP_SEC_CONFIG			15
#define DEEP_SLEEP_SEC_TRAINING_PAUSE	60

// modes for green led (int and not bit mode)
#define LED_OFF							0
#define SLOW_FLASHING_LED				1
#define FAST_FLASHING_LED				2
#define HEARTBEAT_LED					3
#define SHORT_HEARTBEAT_LED				4

// actual green led
#define LED9							8

// orange led effect configuration (during training)
#define ORANGE_LED_EFFECT_STEPS			5
#define ORANGE_LED_EFFECT_DELAY			80	// effect delay in ms between steps
#define ORANGE_LED_EFFECT_PERIOD 		3

// define bit mask for all available and specific LEDs on PORTB and PORTD
#define LED_PORTB						0x02
#define LED_PORTD						0xFF
#define LED_PORTD_RED					0x1F
#define LED_PORTD_ORANGE				0xE0

// define bit mask for switches T1..3, all PORTC
#define SWITCH_T1						0
#define SWITCH_T2						1
#define SWITCH_T3						2
#define NUM_SWITCHES					3
#define SWITCH_MASK						0x07

// define different phases, bit mode
#define PHASE_MAINPROG					0	// something in main loop between different phases
#define PHASE_CONFIG					1
#define PHASE_TRAINING					2
#define PHASE_ACTIVE					3
#define PHASE_REST						4
#define PHASE_PAUSE						5
#define PHASE_SHOW_results				6
#define PHASE_RESET						7

#define CONFIG_INTERVAL_CNT				0	// counter for configuration of interval length
#define CONFIG_NR_INTERVAL_ACTIVE		1	// number of intervals in "active" phase
#define CONFIG_NR_INTERVAL_REST			2	// number of intervals in "rest" phase
#define CONFIG_NR_CYCLES				3	// number of cycles to perform ACT/REST phases
#define NUM_CONFIG_PARAMS				4

#define DEBOUNCE_THRESHOLD				50	// if x ms button high in a row -> button pressed

#endif