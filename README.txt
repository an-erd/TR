PURPOSE
=======
The function of TR is to have a tiny gym timer, which can give you intervals for active and rest
phases during gym or any other sport. Is has 4 buttons (1 reset button, and three functional
buttons T1..3 ), 9 leds (5 red as a progress bar, 3 orange for current phase active or rest, 
resp. (or during config), and a green heartbeat led.)
It is build on an ATMEL ATMEGA328P, but should work on any other similar MCU, too.
The power is assumed to be 3V. The code is optimized for power saving, but since the leds draw a
pretty high load, it is - by system - still consuming around 10 mA on average during training.

The program flow exist of the following steps:
- initialization (I/O pins, timer, ...)
- configuration routine
- training routine

During the phases, to control is done with the buttons, and the leds change the mode accordingly.
The specific status and control is shown in the list below.

The program itself makes heavy use of two timer, and pin change interrupts. The logic itself, in 
particular waiting for a counter to run down and to give effects and progress with the leds is
more or less completly delivered by the relevant interrupts. 


During WAITING MODE
====================
	LED PB1/green		-> slow flashing means "waiting for command"
	T1					-> (nothing yet)
	T2					-> go into config mode
	T3					-> GO! (training)

	LED PD0..7/red/ora	-> off
	LED PB1/green		-> short heartbeat

During CONFIG
=============
	T1					-> increase config parameter value, wrap-arolund 5 -> 1 or 0 resp.
	T2					-> switch to next config parameter, wrap-around is done
	T3					-> GO! (training)

	LED PD0..4/red		-> show configurated value
	LED PD5..7/orange	-> show configuration step (000/INT, 100/ACT, 110/REST, 111/cycles)
	LED PB1/green		-> fast flashing

During TRAINING
===============
	T1					-> (nothing yet)
	T2					-> Stop training and go in "waiting mode", see main loop
	T3					-> Pause training

	LED PD0..4/red		-> show progress in current phase, current step flashes, 1-5 leds
	LED PD5..7/orange	-> show LR or RL cascade to show phase ACTIVE or REST, resp.
	LED PB1/green		-> heartbeat

During PAUSE
============
	LED PD0..4/red		-> freeze current status 
	LED PD5..7/orange	-> freeze current status
	LED PB1/green		-> slow flashing

	T2					-> Stop training/pause and go in "waiting mode", see main loop
	T3					-> end pause, 

TODO and optimization
=====================
- none yet


Timer calculation/values
========================
- 8 MHz quartz, fuse CKDIV8 disabled, F_CPU=8000000UL
- timer0: 8 MHz,  8bit, 1/64  prescale -> 1 KHz (OCR0A   125 ticks / .001s=1ms)
- timer1: 8 MHz, 16bit, 1/256 prescale -> 1 Hz  (OCR1A 31250 Ticks / 1s, OCR1B 3125 Ticks / .1s)

- 8 MHz quartz, fuse CKDIV8 set, F_CPU=1000000UL
- timer0: @F_CPU/8, tick values stay unchanged
- timer1: @F_CPU/64, 1 Hz: OCR1A = 15625, 10 Hz: OCR1B =  1562

FUSE Settings are:
==================
BODLEVEL = 1V8
RSTDISBL = [ ]
DWEN = [ ]
SPIEN = [X]
WDTON = [ ]
EESAVE = [X]
BOOTSZ = 2048W_3800
BOOTRST = [ ]
CKDIV8 = [X]
CKOUT = [ ]
SUT_CKSEL = EXTXOSC_8MHZ_XX_16KCK_14CK_65MS

EXTENDED = 0xFE (valid)
HIGH = 0xD1 (valid)
LOW = 0x7F (valid)

Compiler symbols defined
========================
F_CPU=1000000UL
_PRODUCTION_TEST_ROUTINE_
_ALL_LED_ON_FOR_5_SEC_
