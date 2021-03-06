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
#include <avr/wdt.h>
#include <util/delay.h>

#include "bitoperations.h"
#include "timerruler_struct.h"
#include "timerruler_const.h"
#include "timerruler_led.h"

uint8_t						config_params_ee[NUM_CONFIG_PARAMS] EEMEM	= {1, 4, 4, 3};
volatile static	uint8_t		PINChistory = 0x00;

void main(void) __attribute__((noreturn));

// global variables
volatile sGlobalStatus program_status = {
	.phase							= PHASE_RESET,
	.current_led_step				= 0,
	.buttons						= {{DEBOUNCE_THRESHOLD,0,0,0,},{DEBOUNCE_THRESHOLD,0,0,0,},{DEBOUNCE_THRESHOLD,0,0,0,},},
	.green_led_mode					= LED_OFF,
	.green_led_max_cycle			= 0,
	.green_led_current_cycle		= 0,
	.green_led_OCR1B_basis			= 0,
	.led_effect_ongoing				= LED_OFF,
	.effect_led_ms_counter			= LED_EFFECT_DELAY,
	.effect_led_current_step		= 0,
	.effect_led_direction			= ALL_LED_CENTER_EFFECT_LR,
	.orange_led_period				= 0,
	.backward_cnt_sec_to_deep_sleep = DEEP_SLEEP_SEC_MAINPROG,
};

ISR ( TIMER0_COMPA_vect )
{
	volatile uint8_t	button_backward_cnt = NUM_SWITCHES;
	volatile uint8_t	PORTD_modified;
		
	// fired every 1ms gone

	// button debounce mechanism for T1..3 (PC0..2)
	// - the button is assumed to be stable if "pressed" status occurs 50x (=50ms) in a row
	// - temp_state_button_pressed is set in pin change interrupt routine (ISR (PCINT1_vect))
	//
	// - program flow
	//		a) if not temp_pressed (filled from ISR) 
	//		a1)	-> do: reset counter, and clear the "wait for release" bit
	//		a2) -> else: (if not "waiting for release", inc in-a-row-counter and
	//			if threshold reached -> set the "button pressed" flag
	while (button_backward_cnt--) {
		if ( !program_status.buttons[button_backward_cnt].temp_state_button_pressed ) {
			program_status.buttons[button_backward_cnt].backward_cnt_high_in_a_row = DEBOUNCE_THRESHOLD;
			program_status.buttons[button_backward_cnt].temp_wait_for_button_release = 0;		
		} else {
			if (! program_status.buttons[button_backward_cnt].temp_wait_for_button_release) {
				if (! --program_status.buttons[button_backward_cnt].backward_cnt_high_in_a_row ){
					program_status.buttons[button_backward_cnt].button_pressed = 1;					// debounce interval gone -> stable
					program_status.buttons[button_backward_cnt].temp_wait_for_button_release = 1;
				}
			}
		}
	}

	// the led effects are done w/TIMER0/A, delay between effects w/TIMER1/A
	if (program_status.effect_led_current_step){
		switch (program_status.led_effect_ongoing){
			case LED_EFFECT_ORANGE:
				if(program_status.phase & BIT(PHASE_TRAINING)){
			 		if (! --program_status.effect_led_ms_counter){
 						program_status.effect_led_ms_counter = LED_EFFECT_DELAY;
		 				program_status.effect_led_current_step--;
 
	 					PORTD_modified = PORTD;
 						bit_clear(PORTD_modified, LED_PORTD_ORANGE);
	 					bit_set(PORTD_modified, ledArrayOrangeEffect[program_status.effect_led_current_step][(bit_get(program_status.phase, BIT(PHASE_ACTIVE))) ? 0 : 1] );	
 						PORTD = PORTD_modified;
 					}
 				}
				break;
			case LED_EFFECT_ALL:
				if (! --program_status.effect_led_ms_counter){
					program_status.effect_led_ms_counter = LED_EFFECT_DELAY;
					program_status.effect_led_current_step--;
					PORTD = ledArrayAllEffect[program_status.effect_led_current_step][program_status.effect_led_direction];
				}
 				break;	
			case LED_EFFECT_CENTER:
				if (! --program_status.effect_led_ms_counter){
					program_status.effect_led_ms_counter = LED_EFFECT_DELAY;
					program_status.effect_led_current_step--;
					PORTD = ledArrayAllCenterEffect[program_status.effect_led_current_step];
				}
 				break;
			case LED_OFF:
				break;
			default:
				break;
		}
	}
}

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
	
	if ( bit_get(program_status.phase, BIT(PHASE_TRAINING))){
		if (! --program_status.backward_counter_sec_to_go){
			bit_set(PORTD, BIT(program_status.current_led_step));
		} else {
			bit_flip(PORTD, BIT(program_status.current_led_step));
		}
		
		if (! --program_status.orange_led_period) {
			program_status.orange_led_period		= ORANGE_LED_EFFECT_PERIOD;
			program_status.effect_led_current_step	= ORANGE_LED_EFFECT_STEPS;	// fire effect w/orange leds
		}
	}
			
	if ( bit_get(program_status.phase, BIT(PHASE_MAINPROG)|BIT(PHASE_CONFIG)|BIT(PHASE_PAUSE))){
		--program_status.backward_cnt_sec_to_deep_sleep;
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
				OCR1B  = program_status.green_led_OCR1B_basis;	// reinitialize compare value for next cycle to first hit
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
	
	changedbits = PINC ^ PINChistory;
	PINChistory = PINC;

	if(bit_get(changedbits, BIT(PINC0))){
		program_status.buttons[0].temp_state_button_pressed = !bit_get(PINC, BIT(PINC0));
	}
	if(bit_get(changedbits, BIT(PINC1))){
		program_status.buttons[1].temp_state_button_pressed = !bit_get(PINC, BIT(PINC1));
	}
	if(bit_get(changedbits, BIT(PINC2))){
		program_status.buttons[2].temp_state_button_pressed = !bit_get(PINC, BIT(PINC2));
	}
}

void init_global_vars(void)
{
	program_status.phase = PHASE_RESET;
}

void init_io_pins(void)
{
	// power consumption 480 uA in IDLE mode -> 20.8 uA PWR_DOWN
	DDRB  = BIT(DDB1);				// PB1 as output, all other as input
	PORTB = 0xFD;					// all input w/pull-ups
	DDRC  = 0x00;					// all port C as input
	PORTC = 0x7F;					// PC0..2 used for switches, but set all w/pull-ups
	DDRD  = 0xFF;					// all port D as output (all PD0..7 used for LEDs)
	
	bit_set(PCICR,  BIT(PCIE1));							// pin change interrupt enable
	bit_set(PCMSK1, BIT(PCINT8)|BIT(PCINT9)|BIT(PCINT10) );	// Enable pin change detection

	PINChistory = PINC;
}

void init_timers(void)
{
	// prepare timer0 and timer1
	// - timer0a: switch debounce (1 KHz interrupts), and 3 orange led effect
	// - timer1a: training phase measurement (1 HZz) and 5 red led progress bar 
	// - timer1b: used together with timer1a to get a GREEN LED heartbeat (.1ms/.9s blink), of green flashing led

	// timer0
	bit_set(TCCR0A, BIT(WGM01));				// Configure timer0 for CTC mode,
	bit_set(TCCR0B, BIT(CS01));					// Start timer @F_CPU/8
	OCR0A	= 124;								// 8 MHz, 8bit, CKDIV8 set, 1/8 prescale -> 1 KHz (125 ticks / .001s=1ms
	bit_set(TIMSK0, BIT(OCIE0A));				// Enable Timer/Counter Compare Match A interrupt
	
	// timer1
	bit_set(TCCR1B, BIT(WGM12)|BIT(CS11)|BIT(CS10)); // Configure timer1 for CTC mode, start timer @F_CPU/64
	OCR1A	= 15625; 							// 8 MHz, 16bit, CKDIV8 set, 1/64 prescale -> 1 Hz (= 15625 Ticks/ 1s)
	OCR1B	=  1562;							// and 1562 Ticks / .1s
	bit_set(TIMSK1, BIT(OCIE1A)|BIT(OCIE1B)); 	// Enable Timer/Counter Compare Match interrupt on both channel A and B
}

void restart_timer1(void)
{
	uint8_t history_TCCR1B = TCCR1B & (BIT(CS10)|BIT(CS11)|BIT(CS12));

	bit_clear(TCCR1B, history_TCCR1B);	// stop clock
	TCNT1 = 0;							// reset TCNT1 counter
	led_set(LED9, OFF);
	OCR1B = program_status.green_led_OCR1B_basis;	// set correct OCR1B compare value
	bit_set(TCCR1B, history_TCCR1B);			// start clock
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
			program_status.green_led_OCR1B_basis	= 1953;
			break;
		case SLOW_FLASHING_LED:
			// 1 Hz
			program_status.green_led_mode = SLOW_FLASHING_LED;
			// .green_led_max_cycle and .green_led_current_cycle not necessary (only one cycle)
			program_status.green_led_OCR1B_basis = 7812;
			break;
		case HEARTBEAT_LED:
			// 1 cycle: 0 -> .9s OFF.1s ON
			program_status.green_led_mode = HEARTBEAT_LED;
			// .green_led_max_cycle and .green_led_current_cycle not necessary (only one cycle)
			program_status.green_led_OCR1B_basis = 14062;
			break;
		case SHORT_HEARTBEAT_LED:
			// just blink very short...
			program_status.green_led_mode = SHORT_HEARTBEAT_LED;
			// .green_led_max_cycle and .green_led_current_cycle not necessary (only one cycle)
			program_status.green_led_OCR1B_basis = 15400;
			break;
		default:
			break;
	}
	restart_timer1();
}

void set_led_effect(const uint8_t led_effect)
{
	switch(led_effect){
		case LED_EFFECT_ORANGE:
			program_status.orange_led_period		= 0;
			program_status.effect_led_current_step	= 0;
			program_status.led_effect_ongoing		= LED_EFFECT_ORANGE;
			break;
		case LED_EFFECT_ALL:
			program_status.orange_led_period		= 0;
			program_status.effect_led_current_step	= 0;
			program_status.led_effect_ongoing		= LED_EFFECT_ALL;
			break;
		case LED_EFFECT_CENTER:
			program_status.orange_led_period		= 0;
			program_status.effect_led_current_step	= 0;
			program_status.led_effect_ongoing		= LED_EFFECT_CENTER;
			break;
		case LED_OFF:
			program_status.orange_led_period		= 0;
			program_status.effect_led_current_step	= 0;
			program_status.led_effect_ongoing		= LED_OFF;
			break;
		default:
			break;
	}
}

void set_led_effect_all_direction(const uint8_t led_effect_direction)
{
	program_status.effect_led_direction = led_effect_direction;
}

void start_led_effect(void)
{
	switch(program_status.led_effect_ongoing){
		case LED_EFFECT_ORANGE:
			program_status.orange_led_period		= ORANGE_LED_EFFECT_PERIOD;
			break;
		case LED_EFFECT_ALL:
			program_status.effect_led_current_step	= ALL_LED_EFFECT_STEPS;
			break;
		case LED_EFFECT_CENTER:
			program_status.effect_led_current_step	= ALL_LED_CENTER_EFFECT_STEPS;
			break;
		default:
			break;
	}
}

void all_led_effect_and_wait_to_complete(const uint8_t led_effect_direction)
{
	uint8_t tmp_phase = program_status.phase;
	
	bit_clear(program_status.phase, BIT(PHASE_TRAINING));
	set_led_effect(LED_EFFECT_ALL);
	set_led_effect_all_direction(led_effect_direction);
	start_led_effect();
	do {} while (program_status.effect_led_current_step);
	set_led_effect(LED_OFF);
	program_status.phase = tmp_phase;
}

void center_led_effect_and_wait_to_complete(void)
{
	uint8_t tmp_phase = program_status.phase;
	
	bit_clear(program_status.phase, BIT(PHASE_TRAINING));
	set_led_effect(LED_EFFECT_CENTER);
	start_led_effect();
	do {} while (program_status.effect_led_current_step);
	set_led_effect(LED_OFF);
	program_status.phase = tmp_phase;
}

void enable_ppr(void)
{
	power_adc_disable();				// Analog to Digital Converter module
	power_spi_disable();				// Serial Peripheral Interface module
	power_timer0_enable();				// timer0 module
	power_timer1_enable();				// timer1 module
	power_timer2_disable();				// timer2 module
	power_twi_disable();				// Two Wire Interface module
	power_usart0_disable();				// USART module
	// Turn off brownout detection
}

void touch_deep_sleep_counter(void)
{
	if(bit_get(program_status.phase, BIT(PHASE_MAINPROG))){
		program_status.backward_cnt_sec_to_deep_sleep = DEEP_SLEEP_SEC_MAINPROG;
	} else if(bit_get(program_status.phase, BIT(PHASE_CONFIG))){
		program_status.backward_cnt_sec_to_deep_sleep = DEEP_SLEEP_SEC_CONFIG;
	} else if(bit_get(program_status.phase, BIT(PHASE_PAUSE))){
		program_status.backward_cnt_sec_to_deep_sleep = DEEP_SLEEP_SEC_TRAINING_PAUSE;
	}
}

void go_to_appropriate_sleep_mode(void)
{
	sleep_bod_disable();
	sleep_mode();
	if (! program_status.backward_cnt_sec_to_deep_sleep){
		volatile uint8_t preserve_PIND = PIND;
		bit_clear(PORTD, LED_PORTD);
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		sleep_bod_disable();
		sleep_mode();
		set_sleep_mode(SLEEP_MODE_IDLE);
		PIND = preserve_PIND;
		touch_deep_sleep_counter();
	}
}

void read_permanent_config_params()
{
	eeprom_read_block((void*) program_status.config_params, (const void*) config_params_ee, NUM_CONFIG_PARAMS);
}

void perform_phase_config(void)
{
	uint8_t immediate_go		= 0;
	uint8_t loop_counter		= 0;
	uint8_t tmp_config_value	= 0;

	const uint8_t	config_params_min_max[NUM_CONFIG_PARAMS][2] = {{1,5},{1,5},{1,5},{0,5}};
	
	program_status.phase = BIT(PHASE_CONFIG);		// set phase to CONFIG, no "|" necessary, just set
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
	do {
		tmp_config_value = program_status.config_params[loop_counter];
		PORTD = ledArrayRedCascade_LR[tmp_config_value] | ledArrayOrangeCascade_LR[loop_counter];
		touch_deep_sleep_counter();		
		
		// wait for key pressed
		while (! program_status.buttons[0].button_pressed && ! program_status.buttons[1].button_pressed
			&& !  program_status.buttons[2].button_pressed ){
			go_to_appropriate_sleep_mode();
		}
		
		if (program_status.buttons[0].button_pressed){
			program_status.buttons[0].button_pressed = 0;
			tmp_config_value++;
			if (tmp_config_value > config_params_min_max[loop_counter][1]){
				tmp_config_value = config_params_min_max[loop_counter][0];
			}
			PORTD = ledArrayRedCascade_LR[tmp_config_value] | ledArrayOrangeCascade_LR[loop_counter];
			program_status.config_params[loop_counter] = tmp_config_value;	// write config back
		} 
		if (program_status.buttons[1].button_pressed){
			// T2 hit -> next configuration parameter, and wraparound
			program_status.buttons[1].button_pressed = 0;
			loop_counter++;
			loop_counter %= NUM_CONFIG_PARAMS;
		}
		if (program_status.buttons[2].button_pressed){
			// T3 hit -> GO! with this configuration
			program_status.buttons[2].button_pressed = 0;
			immediate_go++;
		}
	} while (! immediate_go);

	set_green_led_mode(LED_OFF);
	led_set_all(OFF);
	
	program_status.phase = BIT(PHASE_MAINPROG);
}

void store_config_params_permanently()
{
	cli();
	eeprom_busy_wait();
	eeprom_update_block((const void*) program_status.config_params, (void*) config_params_ee, NUM_CONFIG_PARAMS);
	sei();
}

void perform_phase_config_calculation(void)
{
	uint8_t const temp_interval_basis_sec = INTERVAL_LENGTH * program_status.config_params[CONFIG_INTERVAL_CNT];
	uint8_t temp_multiplied_interval_length;
	
	// current steps blinks .5 Hz (on/off in 2 secs), completed steps on,
	// orange leds PB5..7 show phase (ACTIVE: L->R, REST: R->L)

	// calculation of red leds PB0..4 (L->R)
	// for ACTIVE phase:
	temp_multiplied_interval_length =
		temp_interval_basis_sec * program_status.config_params[CONFIG_NR_INTERVAL_ACTIVE];
	program_status.led_steps_threshold[0][0] = temp_multiplied_interval_length / 5;
	program_status.led_steps_threshold[1][0] =
		program_status.led_steps_threshold[0][0] + temp_multiplied_interval_length % 5;
	// for REST phase:
	temp_multiplied_interval_length =
		temp_interval_basis_sec * program_status.config_params[CONFIG_NR_INTERVAL_REST];
	program_status.led_steps_threshold[0][1] = temp_multiplied_interval_length / 5;
	program_status.led_steps_threshold[1][1] =
		program_status.led_steps_threshold[0][1] + temp_multiplied_interval_length % 5;
}

void perform_phase_training(void)
{
	uint8_t		cycle_counter				= 0;
	uint8_t		in_cycle_steps_done			= 0;
	uint8_t		exit_training_immediately	= 0;			// T2 pressed, back to config
	
	bit_flip(program_status.phase, BIT(PHASE_MAINPROG)|BIT(PHASE_TRAINING));
	
	set_led_effect(LED_OFF);
	set_green_led_mode(HEARTBEAT_LED);
		
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
	do { 
		cycle_counter++;
		in_cycle_steps_done = 0;

		bit_set(program_status.phase, BIT(PHASE_ACTIVE));
		bit_clear(program_status.phase, BIT(PHASE_REST));		
		
		do { 
			// in-cycle 2-step loop
			// Complete an ACTIVE or REST phase:
			//	a) update red leds (PD0..4) for in-phase progress regularly
			//	b) update orange leds (PD5..7) for training phase ACTIVE (L->R)/REST (R->L)
			//	c) green led stays in heartbeat mode
			bit_clear(PORTD, LED_PORTD_RED);
			all_led_effect_and_wait_to_complete(in_cycle_steps_done);
			set_led_effect(LED_EFFECT_ORANGE);
			start_led_effect();
			
			// 5 leds, steps 0..4 to do, e.g., "3" means: LED0..2 on, LED3 flash, LED4 off
			for (program_status.current_led_step = 0; program_status.current_led_step < 5; program_status.current_led_step++){
				// .backward_counter_sec_to_go to be decreased by timer1/A, threshold values in array
				program_status.backward_counter_sec_to_go = 
					program_status.led_steps_threshold[(program_status.current_led_step ? 0 : 1)][in_cycle_steps_done];
				bit_set(PORTD, ledArrayRedCascade_LR[program_status.current_led_step]);
				touch_deep_sleep_counter();
				do { 
					// wait for timer run-down...
					// red solid, flashing and off -> update through timer1/A regularly
					// orange leds showing training mode -> update through timer1/A regularly
					go_to_appropriate_sleep_mode();

					// if button T1 pressed -> do nothing 
					if (program_status.buttons[0].button_pressed){
						program_status.buttons[0].button_pressed = 0;
					}
					
					// if button T2 pressed -> stop training, and go back to config phase
					if (program_status.buttons[1].button_pressed){
						program_status.buttons[1].button_pressed = 0;
						exit_training_immediately = 1;
					}
						
					// if button T3 pressed -> toggle PAUSE mode
					if (program_status.buttons[2].button_pressed){
						program_status.buttons[2].button_pressed = 0;
						
						if (bit_get(program_status.phase, BIT(PHASE_TRAINING)))	{
							// stop decreasing .backward_counter_sec_to_go/orange leds, but start deep sleep backward counter 
							bit_flip(program_status.phase, BIT(PHASE_TRAINING)|BIT(PHASE_PAUSE));
							touch_deep_sleep_counter();
							set_green_led_mode(SLOW_FLASHING_LED);
						} else if (program_status.phase & BIT(PHASE_PAUSE)){
							// enable decreasing .backward_counter_sec_to_go/orange leds, but stop deep sleep backward counter 
							// deep sleep backward counter will be stopped
							bit_flip(program_status.phase, BIT(PHASE_PAUSE)|BIT(PHASE_TRAINING));
							set_green_led_mode(HEARTBEAT_LED);
						}
					}
				} while (! exit_training_immediately && program_status.backward_counter_sec_to_go);		// will not be decreased in ISR if in PAUSE mode, so no problem
			}
			
			bit_flip(program_status.phase, BIT(PHASE_ACTIVE)|BIT(PHASE_REST));
			in_cycle_steps_done++;
		} while (! exit_training_immediately && in_cycle_steps_done < 2);	// perform 2nd cycle or leave (if this was the 2nd cycle or immediate exit)

	// exit while-loop, if "exit_training_immediately=1" or number_cycles_reached and not repeat endlessly
	} while ( ! exit_training_immediately 
			&& ( (! program_status.config_params[CONFIG_NR_CYCLES])	// true, if repeat endlessly is configured
			|| (cycle_counter < program_status.config_params[CONFIG_NR_CYCLES])));		// or all cycles completed

	// all cycles completed 
	if(!exit_training_immediately){
		center_led_effect_and_wait_to_complete();
	}

	// cleanup 
	set_green_led_mode(LED_OFF);
	led_set_all(OFF);
	set_led_effect(OFF);

	bit_clear(program_status.phase, BIT(PHASE_TRAINING)|BIT(PHASE_ACTIVE)|BIT(PHASE_REST));
	bit_set(program_status.phase, BIT(PHASE_MAINPROG));
}

void main(void)
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
	init_global_vars();
	init_io_pins();
	init_timers();
	wdt_disable();
	enable_ppr();						// set PRR (power reduction register)
	sei();

	program_status.phase = BIT(PHASE_MAINPROG);
	
#ifdef _PRODUCTION_TEST_ROUTINE_
	// flash of all leds to check after production
	led_set_all(ON); _delay_ms(10); led_set_all(OFF); _delay_ms(200);
#endif //_PRODUCTION_TEST_ROUTINE_

#ifdef _ALL_LED_ON_FOR_5_SEC_
	// to measure power consumption
	led_set_all(ON); _delay_ms(5000);led_set_all(OFF); _delay_ms(200);
#endif // _ALL_LED_ON_FOR_5_SEC_

#ifdef _INCREASE_NUM_LED_ON_
	// to measure power consumption
 	uint8_t temp_cnt;
 	led_set_all(OFF);
 	for (temp_cnt=0;temp_cnt<9;temp_cnt++)
 	{
 		_delay_ms(5000);
 		led_set(temp_cnt, ON);
 	}
#endif // _INCREASE_NUM_LED_ON_

	read_permanent_config_params();
	perform_phase_config_calculation();
	set_sleep_mode(SLEEP_MODE_IDLE);	// two different stages will be used, IDLE if heartbeat is on, and later PWR_DOWN
	set_green_led_mode(SHORT_HEARTBEAT_LED);
	
	while (1)	{
		touch_deep_sleep_counter();
		do {
			go_to_appropriate_sleep_mode();
		} while ( !program_status.buttons[0].button_pressed && !program_status.buttons[1].button_pressed 
			&& !program_status.buttons[2].button_pressed);
		
		// button T1 -> do nothing 
		if (program_status.buttons[0].button_pressed){
			program_status.buttons[0].button_pressed = 0;
		}
		
		// button T2 -> config
		if (program_status.buttons[1].button_pressed){
			program_status.buttons[1].button_pressed = 0;
			perform_phase_config();
			store_config_params_permanently();
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
			perform_phase_training();	
			
			program_status.phase = BIT(PHASE_MAINPROG);
			set_green_led_mode(SHORT_HEARTBEAT_LED);
		}
	}
}