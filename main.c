/*
 * TimerRuler.c
 *
 * Created: 22.11.2015 10:51:17
 * Author : AKAEM
 */ 

/* atmega328p ports used:
	 PC0	Switch T1	PCINT8		0
	 PC1	Switch T2	PCINT9		1
	 PC2	Switch T3	PCINT10		2
	(PC4	I2C SDA		unused)
	(PC5	I2C SCL		unused)
	 PD0	LED1 RED				0
	 PD1	LED2 RED				1
	 PD2	LED3 RED				2
	 PD3	LED4 RED				3
	 PD4	LED5 RED				4
	 PD5	LED6 ORANGE				5
	 PD6	LED7 ORANGE				6
	 PD7	LED8 ORANGE				7
	 PB1 	LED9 GREEN				8
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <util/delay.h>

#include "bitoperations.h"
#include "timerruler_struct.h"
#include "timerruler_const.h"
#include "timerruler_led.h"

uint8_t EEMEM config_params_ee[CONFIG_NR_CYCLES+1] = {1, 4, 4, 3};

// global variables
volatile sGlobalStatus program_status = {
	.phase						= PHASE_RESET,
	.current_led_step			= 0,
	.buttons					= { {SWITCH_PRELOAD, 0, 0, 0,}, {SWITCH_PRELOAD, 0, 0, 0,}, {SWITCH_PRELOAD, 0, 0, 0,},},
	.green_led_mode				= LED_OFF,
	.green_led_max_cycle		= 0,
	.green_led_current_cycle	= 0,
	.green_led_OCR1B_basis	= 0,
	.orange_led_counter			= ORANGE_LED_EFFECT_DELAY,	// delay when to update orange leds during effect
	.orange_led_current_step	= 0,						// updated in ISR, etc. (decrease)
	.orange_led_max_step		= ORANGE_LED_EFFECT_STEPS,	// const
	.orange_led_period			= 0,						// set/reload to ORANGE_LED_EFFECT_PERIOD to start
	.PINChistory = 0x00,
};

ISR ( TIMER0_COMPA_vect )
{
	volatile uint8_t	button_cnt = NUM_SWITCHES;
	
	// interrupt routine fired every 1ms gone

	// button debounce mechanism for T1..3 (PC0..2)
	// - the button is assumed to be stable if "pressed" status occurs 50x (=50ms) in a row
	// - temp_state_button_pressed is set in pin change interrupt routine (ISR (PCINT1_vect))
	// - prepared for "auto repeat" mode, i.e., if the button is not release. 
	//		Currently we do have some problems with a non deterministic behavior, thus the mode is switched off.
	//		TODO remove the .temp_wait_for_button_release and check again.
	//
	// - the code is essentially the same for all buttons, fired every 1ms by TIMER0_COMPA_vect
	//		a) if not remp_pressed (filled from ISR) 
	//		a1)	-> do: reset counter, and clear the "wait for release" bit
	//		a2) -> else: (if not "waiting for release", inc in-a-row-counter and
	//			if threshold reached -> set the "button pressed" flag
	while (button_cnt--) {
		if ( !program_status.buttons[button_cnt].temp_state_button_pressed ) {
			program_status.buttons[button_cnt].counter_high_in_a_row		= SWITCH_PRELOAD;
			program_status.buttons[button_cnt].temp_wait_for_button_release = 0;		
		} else {
			if (! program_status.buttons[button_cnt].temp_wait_for_button_release) {		// deactivate auto repeat for the moment	TODO!
				if ( ++program_status.buttons[button_cnt].counter_high_in_a_row == SWITCH_COMPARE ){
					program_status.buttons[button_cnt].button_pressed = 1;					// button pressed and stable, debounce interval gone
					program_status.buttons[button_cnt].temp_wait_for_button_release = 1;		// deactivate auto repeat for the moment	TODO!
					program_status.buttons[button_cnt].counter_high_in_a_row = 0;			// start from 0 to get SWITCH_COMPARE ms until next button pressed event (auto repeat)
				}
			}
		}
	}


	// the orange led effect is done with timer0/A
	if ( program_status.phase & BIT(PHASE_TRAINING) ){
		if (! --program_status.orange_led_counter){
			program_status.orange_led_counter = ORANGE_LED_EFFECT_DELAY;
			if(program_status.orange_led_current_step--) {
				// copy PORTD, change orange led relevant part, and write back to PORTD (to avoid flickering... necessary? TODO)
				bit_clear(PORTD, LED_PORTD_ORANGE);
				bit_set(PORTD, ledArrayOrangeEffect[program_status.orange_led_current_step][(bit_get(program_status.phase, BIT(PHASE_ACTIVE))) ? 0 : 1] );
			}
		}
	}
}

// Interrupt for Timer/Counter1 Compare Match A
ISR ( TIMER1_COMPA_vect )
{
	// 1s gone
	
	// green led tasks
	switch (program_status.green_led_mode) {
		case HEARTBEAT_LED:
			led_set(LED9, OFF);		// no change in compare value, hit comp B in next cycle
			break;
		case SHORT_HEARTBEAT_LED:
			led_set(LED9, OFF);		// no change in compare value, hit comp B in next cycle
			break;
		case FAST_FLASHING_LED:
			led_flip(LED9);			// OCR1B and .green_led_current_cycle already set in comp B
			break;
		case SLOW_FLASHING_LED:
			led_set(LED9, OFF);		// no change in compare value, hit comp B in next cycle
			break;
		case LED_OFF:				// nothing to do, should be off
			break;
		default:
			break;
	}
	
	if ( program_status.phase & BIT(PHASE_TRAINING)){
		if (! --program_status.backward_counter_sec_to_go){
			bit_set(PORTD, BIT(program_status.current_led_step));
		} else {
			bit_flip(PORTD, BIT(program_status.current_led_step));
		}
		
		if (! --program_status.orange_led_period) {
			program_status.orange_led_current_step = program_status.orange_led_max_step;
			program_status.orange_led_period = ORANGE_LED_EFFECT_PERIOD;
		}
	}
}

ISR ( TIMER1_COMPB_vect )
{
	// update only green led 
	switch (program_status.green_led_mode) {
		case HEARTBEAT_LED:
			led_set(LED9, ON);		// no change in compare value, hit comp A in next cycle
			break;
		case SHORT_HEARTBEAT_LED:
			led_set(LED9, ON);		// no change in compare value, hit comp A in next cycle
			break;
		case FAST_FLASHING_LED:
			led_flip(LED9);
			if (--program_status.green_led_current_cycle){
				OCR1B += program_status.green_led_OCR1B_basis;	// prepare for next hit/cycle
			} else {
				OCR1B = program_status.green_led_OCR1B_basis;	// reinitialize compare value for next cycle to first hit
				program_status.green_led_current_cycle = program_status.green_led_max_cycle; // reinitialize cycle counter
			}
			break;
		case SLOW_FLASHING_LED:
			led_set(LED9, ON);		// hit comp A in next cycle
			break;
		case LED_OFF: 				// should already be off
			break; 
		default:
			break;
	}
}

ISR ( PCINT1_vect )
{
	volatile uint8_t changedbits;

	// Interrupt routine handles pin chance interrupts for PC0..2
	// if a button changes the state, .temp_state_button_pressed is updated
	// results are evaluated (in particular, debouncing) in ISR (TIMER0_COMPA_vect), 
	// and used throughout the program by program_status.Tx.button_pressed, which then need to be cleared again
	
	changedbits = PINC ^ program_status.PINChistory;
	program_status.PINChistory = PINC;

	if(changedbits & BIT(PINC0)){
		program_status.buttons[0].temp_state_button_pressed = !(PINC & BIT(PINC0));
	}
	if(changedbits & BIT(PINC1)){
		program_status.buttons[1].temp_state_button_pressed = !(PINC & BIT(PINC1));
	}
	if(changedbits & BIT(PINC2)){
		program_status.buttons[2].temp_state_button_pressed = !(PINC & BIT(PINC2));
	}
}

void init_global_vars(void)
{
	program_status.phase = PHASE_RESET;
	
	return;
}

void init_io_pins(void)
{
	DDRB  = BIT(DDB1);				// PB1 as output, all other as input
	DDRC  = 0x00;					// all port C as input  (only PC0..2 used for switch T1..3, I2C (PC4, PC5) not considered yet)
	DDRD  = 0xFF;					// all port D as output (all PD0..7 used for LEDs)
	PORTC = 0xFF;					// PC0..2 used for switches, but set all as input w/pull-up
	
	PCICR  |= BIT(PCIE1);									// pin change interrupt enable
	PCMSK1 |= ( BIT(PCINT8) | BIT(PCINT9) | BIT(PCINT10) );	// Enable pin change detection

	program_status.PINChistory = PINC;
	
	return;	
}

void init_timers(void)
{
	// prepare timer0 and timer1
	// - timer0a: switch debounce (1 KHz interrupts), and 3 orange led effect
	// - timer1a: training phase measurement (1 HZz) and 5 red led progress bar 
	// - timer1b: used together with timer1a to get a GREEN LED heartbeat (.1ms/.9s blink), of green flashing led

	// timer0
	TCCR0A |= BIT(WGM01);						// Configure timer0 for CTC mode,
	TCCR0B |= ( BIT(CS00) | BIT(CS01));			// Start timer @F_CPU/64
	OCR0A = 124;								// 8 MHz, 8bit, 1/64 prescale -> 1 KHz (125 ticks / .001s=1ms
	TIMSK0 |= BIT(OCIE0A);						// Enable Timer/Counter Compare Match A interrupt
	
	// timer1
	TCCR1B |= ( BIT(WGM12) | BIT(CS12)); 		// Configure timer1 for CTC mode, start timer @F_CPU/256
	OCR1A = 31249; 								// 8 MHz, 16bit, 1/256 prescale -> 1 Hz (= 31250 Ticks/ 1s)
	OCR1B =  3124;								// and 3125 Ticks / .1s
	TIMSK1 |= ( BIT(OCIE1A) | BIT(OCIE1B)); 	// Enable Timer/Counter Compare Match interrupt on both channel A and B

	return;
}

void restart_timer1(void)
{
	uint8_t history_TCCR1B = TCCR1B & (BIT(CS10)|BIT(CS11)|BIT(CS12));

	TCCR1B &= ~history_TCCR1B;			// stop clock
	TCNT1 = 0;							// reset TCNT1 counter

	// prepare green led things
	led_set(LED9, OFF);
	OCR1B = program_status.green_led_OCR1B_basis;	// set correct OCR1B compare value
	
	TCCR1B |= history_TCCR1B;			// start clock
			
	return;
}

void set_green_led_mode(uint8_t led_mode)
{
	// set configuration parameters for green led,
	// actual handling is done in ISR for timer1 compare match A/B
	switch (led_mode) {
		case LED_OFF:
			program_status.green_led_mode			= LED_OFF;
			program_status.green_led_max_cycle		= 0;
			program_status.green_led_current_cycle	= 0;
			program_status.green_led_OCR1B_basis	= 0;
			break;
		case FAST_FLASHING_LED:
			// 4 Hz
			program_status.green_led_mode			= FAST_FLASHING_LED;
			program_status.green_led_max_cycle		= 7;
			program_status.green_led_current_cycle	= program_status.green_led_max_cycle;
			program_status.green_led_OCR1B_basis	= 3906;
			break;
		case SLOW_FLASHING_LED:
			// 1 Hz
			program_status.green_led_mode = SLOW_FLASHING_LED;
			// .green_led_max_cycle and .green_led_current_cycle not necessary (only one cycle)
			program_status.green_led_OCR1B_basis = 15625;
			break;
		case HEARTBEAT_LED:
			// 1 cycle: 0 -> .9s/28125 ticks OFF, -> 1s/31250 ticks ON
			program_status.green_led_mode = HEARTBEAT_LED;
			// .green_led_max_cycle and .green_led_current_cycle not necessary (only one cycle)
			program_status.green_led_OCR1B_basis = 28125;
			break;
		case SHORT_HEARTBEAT_LED:
			// just blink very short...
			program_status.green_led_mode = SHORT_HEARTBEAT_LED;
			// .green_led_max_cycle and .green_led_current_cycle not necessary (only one cycle)
			program_status.green_led_OCR1B_basis = 30800;
			break;
		default:
			break;
	}
	
	restart_timer1();

	return;
}

void read_permanent_config_params()
{
	uint8_t local_config_params_from_ee[NUM_CONFIG_PARAMS];
	
	eeprom_read_block( (void*) local_config_params_from_ee, (const void*) config_params_ee, CONFIG_NR_CYCLES+1 );
	
	if (local_config_params_from_ee[0] == 0xFF){
		// for any unknown reason, EEPROM not initialized
		program_status.config_params[CONFIG_INTERVAL_CNT]		= 1;	// set X * 15sec interval
		program_status.config_params[CONFIG_NR_INTERVAL_ACTIVE] = 4;	// set X * INT
		program_status.config_params[CONFIG_NR_INTERVAL_REST]	= 4;	// set X * INT
		program_status.config_params[CONFIG_NR_CYCLES]			= 3;	// repeat endlessly
		} else {
		program_status.config_params[CONFIG_INTERVAL_CNT]		= local_config_params_from_ee[CONFIG_INTERVAL_CNT];
		program_status.config_params[CONFIG_NR_INTERVAL_ACTIVE] = local_config_params_from_ee[CONFIG_NR_INTERVAL_ACTIVE];
		program_status.config_params[CONFIG_NR_INTERVAL_REST]	= local_config_params_from_ee[CONFIG_NR_INTERVAL_REST];
		program_status.config_params[CONFIG_NR_CYCLES]			= local_config_params_from_ee[CONFIG_NR_CYCLES];
	}
	
	return;
}

void perform_phase_config(void)
{
	uint8_t immediate_go = 0;
	uint8_t loop_counter = 0;
	uint8_t tmp_config_value = 0;

	const uint8_t	config_params_min_max[NUM_CONFIG_PARAMS][2] = {{1,5},{1,5},{1,5},{0,5}};
	
	program_status.phase = (1 << PHASE_CONFIG);		// set phase to CONFIG, no "|" necessary, just set
	set_green_led_mode(FAST_FLASHING_LED);			// green led flashing fast -> configuration mode
		
	// The configuration routine is essentially a loop using the same code to complete or update config params,
	// using an index, i.e. 
	//
	// while (! Button T3 pressed)
	// for i in
	//		0 -> CONFIG_INTERVAL_CNT
	//		1 -> CONFIG_NR_INTERVAL_ACTIVE
	//		2 -> CONFIG_NR_INTERVAL_REST
	//		3 -> CONFIG_NR_CYCLES
	// do
	//		update orange leds PD5..7 with 0 to all three leds according to i
	//		press T1 X-times -> red leds PD0..4 show the count according to the click counter
	//		press T2 for next step in "for i" loop and wraparound 0->1->2->3->0
	//			or press T3 to leave configuration mode and GO!
	//	done
	// 
	// Details:
	//		Step 1)		press T1 x times -> x * 15 sec Intervall, store counter in .config_params[1..3]
	//					(red leds show counter 1-5, orange leds off; 5 red -> 5 * 75 sec max)
	//		Step 2-4)	press T1 x time -> store counter in .config_params[1..3]
	//					(red leds show counter 1-5, orange leds 1, 2 or all 3 are on)
// goto LEAVE_FCT;	// DEBUG
	do
	{
		tmp_config_value = program_status.config_params[loop_counter];
		
		// update leds: PORTD = red 1..5 according to counter & orange 1..3 according to loop counter
		PORTD = ledArrayRedCascade_LR[tmp_config_value] | ledArrayOrangeCascade_LR[loop_counter];
				
		// T2 for next config param, T3 for leave configuration and GO!
		while (! program_status.buttons[1].button_pressed && ! program_status.buttons[2].button_pressed)
		{
			if (program_status.buttons[0].button_pressed){
				program_status.buttons[0].button_pressed = 0;
			
				// increase counter and do wraparound, if necessary
				tmp_config_value++;
				if (tmp_config_value > config_params_min_max[loop_counter][1])		// current value > max for config param
					tmp_config_value = config_params_min_max[loop_counter][0];		// -> wraparound, set to min value
			
				// update leds
				PORTD = ledArrayRedCascade_LR[tmp_config_value] | ledArrayOrangeCascade_LR[loop_counter];
			}
		}
		
		// T2 oder T3 has been pressed
		program_status.config_params[loop_counter] = tmp_config_value;	// store value back to config vector
		if (program_status.buttons[1].button_pressed)
		{
			// T2 hit -> next configuration parameter, and wraparound
			program_status.buttons[1].button_pressed = 0;
			loop_counter++;
			loop_counter %= CONFIG_NR_CYCLES+1;		// increase loop counter MOD max value
		} 
		else
		{
			// T3 hit -> GO! with this configuration
			program_status.buttons[2].button_pressed = 0;
			immediate_go++;
		}
		
	} while (! immediate_go);
	
// LEAVE_FCT:	// DEBUG

	set_green_led_mode(LED_OFF);
	led_set_all(OFF);
	
	program_status.phase = (1 << PHASE_MAINPROG);	// set phase to MAINPROG, no "|"
	
	return;	
}

void store_config_params_permanently()
{
	uint8_t local_config_params_to_ee[CONFIG_NR_CYCLES+1];
	
	local_config_params_to_ee[CONFIG_INTERVAL_CNT]			= program_status.config_params[CONFIG_INTERVAL_CNT];
	local_config_params_to_ee[CONFIG_NR_INTERVAL_ACTIVE]	= program_status.config_params[CONFIG_NR_INTERVAL_ACTIVE];
	local_config_params_to_ee[CONFIG_NR_INTERVAL_REST]		= program_status.config_params[CONFIG_NR_INTERVAL_REST];
	local_config_params_to_ee[CONFIG_NR_CYCLES]				= program_status.config_params[CONFIG_NR_CYCLES];

	eeprom_write_block((const void*) local_config_params_to_ee, (void*) config_params_ee, CONFIG_NR_CYCLES+1 );
	
	return;
}

void perform_phase_config_calculation(void)
{
	uint8_t temp_multiplied_interval_length;	// calculate interval length * nr intervals for ACTIVE/REST, resp.
	uint8_t temp_interval_length;				// calculate interval length and remainder
	uint8_t temp_remainder;

	// use the configured configuration values to calculate interval length, etc.
	program_status.interval_basis_sec = INTERVAL_LENGTH * program_status.config_params[CONFIG_INTERVAL_CNT];
	
	// current steps blinks .5 Hz (on/off in 2 secs), completed steps on,
	// orange leds PB5..7 show phase (ACTIVE: L->R, REST: R->L)

	// calculation of red leds PB0..4 (L->R), unrolled loop
	// for ACTIVE phase:
	temp_multiplied_interval_length = 
		program_status.interval_basis_sec * program_status.config_params[CONFIG_NR_INTERVAL_ACTIVE];
	temp_interval_length = temp_multiplied_interval_length / 5;
	temp_remainder       = temp_multiplied_interval_length % 5;
	program_status.led_steps_threshold[0][0] = temp_interval_length;
	program_status.led_steps_threshold[1][0] = temp_interval_length;
	program_status.led_steps_threshold[2][0] = temp_interval_length;
	program_status.led_steps_threshold[3][0] = temp_interval_length;
	program_status.led_steps_threshold[4][0] = temp_interval_length + temp_remainder;
	// for REST phase:
	temp_multiplied_interval_length = 
		program_status.interval_basis_sec * program_status.config_params[CONFIG_NR_INTERVAL_REST];
	temp_interval_length = temp_multiplied_interval_length / 5;
	temp_remainder       = temp_multiplied_interval_length % 5;
	program_status.led_steps_threshold[0][1] = temp_interval_length;
	program_status.led_steps_threshold[1][1] = temp_interval_length;
	program_status.led_steps_threshold[2][1] = temp_interval_length;
	program_status.led_steps_threshold[3][1] = temp_interval_length;
	program_status.led_steps_threshold[4][1] = temp_interval_length + temp_remainder;
	
	return;
}

void perform_phase_training(void)
{
	uint8_t cycle_counter = 0;
	uint8_t in_cycle_steps_done = 0;
	uint8_t exit_training_immediately = 0;			// stop training and go back to config, will occur if T2 is pressed 
	
	// set TRAINING phase
	program_status.phase &= ~BIT(PHASE_MAINPROG);	// clear bit MAINPROG
	program_status.phase |=  BIT(PHASE_TRAINING);	// set bit TRAINIG
	
	// prepare green and orange leds
	set_green_led_mode(HEARTBEAT_LED);								// green led heartbeat mode -> training phase
 	program_status.orange_led_period = ORANGE_LED_EFFECT_PERIOD;	// start orange led after defined period
 	program_status.orange_led_current_step = 0;						// was: program_status.orange_led_max_step;

	// The training routine is essentially a loop using the same code to perform ACTIVE and REST phases, i.e.
	//
	// do
	//		// perform one cycle, i.e. an ACTIVE and a REST training phase measured by in_cycle_steps_done 
	//		do
	//			// perform the two in-cycle steps  
	//			for (i=1 to 5 leds)
	//				// perform a in-cycle step (ACTIVE/REST)
	//				calculate backward counter
	//				do
	//					// timer1/A will decrease
	//				while (backward counter > 0)
	//		while (in_cycle_counter < 2)
	// while (! all cycles completed, or repeat endlessly)
	
	do { // cycle loop
		cycle_counter++;								// first or next cycle
		in_cycle_steps_done = 0;						// increase 0->2 in each cycle (+1 for ACTIVE and REST completed, resp.)
		
		// set/switch REST -> ACTIVE
		program_status.phase &= ~BIT(PHASE_REST);		// clear bit REST
		program_status.phase |=  BIT(PHASE_ACTIVE);	// set bit ACTIVE
		
		do { // in-cycle 2-step loop
			// Complete an ACTIVE or REST phase:
			// a) update red leds (PD0..4) for in-phase progress regularly
			// b) update orange leds (PD5..7) for training phase ACTIVE (L->R)/REST (R->L)
			// c) green led stays in heartbeat mode
			
			// clear red leds
			bit_clear(PORTD, LED_PORTD_RED);

			// fire cascade LR or RL, respectively, to start ACTIVE or REST phase
			// TODO 

			// 5 leds, steps 0..4 to do, e.g., "3" means: LED0..2 on, LED3 flash, LED4 off
			for (program_status.current_led_step = 0; program_status.current_led_step < 5; program_status.current_led_step++)
			{
				// .backward_counter_sec_to_go to be decreased by timer1/A, threshold values in array
				program_status.backward_counter_sec_to_go = 
					program_status.led_steps_threshold[program_status.current_led_step][in_cycle_steps_done];
				
				// update stable red leds
				bit_set(PORTD, ledArrayRedCascade_LR[program_status.current_led_step]);		// remark: ledArray...[0] = 0, so the -1 is done due to array content

				do { 
					// wait for timer run-down...
					// red solid, flashing and off -> update through timer1/A regularly
					// orange leds showing training mode -> update through timer1/A regularly

					// if button T1 pressed -> do nothing 
					if (program_status.buttons[0].button_pressed){
						program_status.buttons[0].button_pressed = 0;
					}
		
					
					// if button T2 pressed -> stop training, and go back to config phase
					if (program_status.buttons[1].button_pressed)
					{
						program_status.buttons[1].button_pressed = 0;
						exit_training_immediately = 1;
					}
						
					// if button T3 pressed -> toggle PAUSE mode
					if (program_status.buttons[2].button_pressed){
						program_status.buttons[2].button_pressed = 0;
						
						if (program_status.phase & BIT(PHASE_TRAINING))
						{
							// switch to PAUSE, and thus decreasing .backward_counter_sec_to_go and orange leds will be stopped
							program_status.phase &= ~BIT(PHASE_TRAINING);		// clear bit TRAINING
							program_status.phase |=  BIT(PHASE_PAUSE);		// set bit PAUSE
							set_green_led_mode(SLOW_FLASHING_LED);
						} else if (program_status.phase & BIT(PHASE_PAUSE)){
							// switch to TRAINING, thus enable decreasing .backward_counter_sec_to_go and orange leds again
							program_status.phase &= ~BIT(PHASE_PAUSE);		// clear bit PAUSE
							program_status.phase |=  BIT(PHASE_TRAINING);		// set bit TRAINING
							set_green_led_mode(HEARTBEAT_LED);
						}
					}
					
				} while (! exit_training_immediately && program_status.backward_counter_sec_to_go);		// will not be decreased in ISR if in PAUSE mode, so no problem
			}
			
			// switch ACTIVE -> REST		
			program_status.phase &= ~BIT(PHASE_ACTIVE);	// clear bit ACTIVE
			program_status.phase |=  BIT(PHASE_REST);		// set bit REST
			in_cycle_steps_done++;
		} while (! exit_training_immediately && in_cycle_steps_done < 2);	// perform 2nd cycle or leave (if this was the 2nd cycle or immediate exit)

	// fire cascade -> another cycle completed
	// TODO

	// exit while-loop, if "exit_training_immediately=1" or number_cycles_reached and not repeat endlessly	(TODO clearify!)
	} while ( ! exit_training_immediately 
		&& ( (! program_status.config_params[CONFIG_NR_CYCLES])	// true, if repeat endlessly is configured
			 || (cycle_counter < program_status.config_params[CONFIG_NR_CYCLES])));		// or all cycles completed

	// fire cascade -> all cycles completed
	// TODO

	// cleanup 
	set_green_led_mode(LED_OFF);
	led_set_all(OFF);

	program_status.phase &= ~BIT(PHASE_TRAINING);			// clear bit TRAINING, ACTIVE, REST
	program_status.phase &= ~BIT(PHASE_ACTIVE);
	program_status.phase &= ~BIT(PHASE_REST);
	program_status.phase |=	 BIT(PHASE_MAINPROG);			// set bit MAINPROG
	
	return;	
}

int main(void)
{
	uint8_t	go_to_training_next = 0;
	
	// overall program sketch
	//	1) initialization (global vars, I/O pins, timers, interrupts, ...)
	//	2) button T1 -> nothing yet
	//	   button T2 -> PHASE CONFIG: user input for interval multiplier, #active and #rest intervals, #cycles
	//	   button T3 -> PHASE TRAINING: perform ACTIVE and REST alternately, until #cycles reached or do it endlessly
	//
	//	also using switches during training: 
	//		- view number cycles achieved		TODO
	//		- view config						TODO
	
	// Initialization
	init_global_vars();					// initialize global variables 
	init_io_pins();						// configure all I/O pins, active pin change interrupts, copy PINC state (should be 0)
	init_timers();						// initialize timer0 and timer1
	sei (); 							// Enable global interrupts
	
	// take EEPROM config and prepare for an immediate go or later config
	read_permanent_config_params();		// read from permanent storage
	perform_phase_config_calculation();

#ifdef _PRODUCTION_TEST_ROUTINE_
	// flash of all leds to check after production
	led_set_all(ON); _delay_ms(10); led_set_all(OFF); _delay_ms(200);
#endif //_PRODUCTION_TEST_ROUTINE_

	// prepare "wait mode" state
	program_status.phase = BIT(PHASE_MAINPROG);
	set_green_led_mode(SHORT_HEARTBEAT_LED);

	while (1)
	{
		// key pressed?
		do 
		{
			// just wait for a key pressed
		} while ( !program_status.buttons[0].button_pressed && !program_status.buttons[1].button_pressed 
			&& !program_status.buttons[2].button_pressed);
		
		// button T1 -> do nothing 
		if (program_status.buttons[0].button_pressed){
			program_status.buttons[0].button_pressed = 0;
		}
		
		// button T2 -> config
		if (program_status.buttons[1].button_pressed){
			program_status.buttons[1].button_pressed = 0;
			
			// Configuration
			perform_phase_config();				// user configuration
			store_config_params_permanently();	// store to permanent storage
			perform_phase_config_calculation();
			
			go_to_training_next = 1;			
		}

		// button T3 -> training
		if (program_status.buttons[2].button_pressed){
			program_status.buttons[2].button_pressed = 0;
			
			go_to_training_next = 1;
		}

		if (go_to_training_next){
			go_to_training_next = 0;
			
			// Training
			perform_phase_training();			// ACTIVE and REST phases, until number of cycles reached, or repeat endlessly
	
			// go to "wait mode" again
			program_status.phase = BIT(PHASE_MAINPROG);	// just to ensure, not necessary? TODO
			set_green_led_mode(SHORT_HEARTBEAT_LED);
		}
	}
	
	return 0;							// return value?
}
