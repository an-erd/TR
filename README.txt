PURPOSE
=======
The function of TR is to have a tiny gym timer, which can give you intervals for active and rest phases during gym 
or any other sport. Is has 4 buttons (1 reset button, and three functional buttons T1..3 ), 9 leds (5 red as a 
progress bar, 3 orange for current phase active or rest, resp (or during config), and a green heartbeat led.)
It is build on an ATMEL ATMEGA328P, but should work on any other similar MCU, too.
The power is assumed to be 3V. The code is optimized for power saving, but since the leds draw a pretty high load,
it is - by system - still consuming around XXX mA.

The program flow exist of the following steps:
- initialization (I/O pins, timer, ...)
- configuration routine
- training routine

During the phases, to control is done with the buttons, and the leds change the mode accordingly. The specific 
status and control is shown in the list below.

The program itself makes heavy use of two timer, and pin change interrupts. The logic itself, in particular waiting 
for a counter to run down and to give effects and progress with the leds is more or less completly delivered by the
relevant interrupts. 


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
	T1					-> increase config parameter value, and do a wrap-arolund 5 -> 1 or 0 resp.
	T2					-> switch to next config parameter, and do a wrap-around if all 4 config params are done
	T3					-> GO! (training)

	LED PD0..4/red		-> show configurated value
	LED PD5..7/orange	-> show configuration step (000/INT, 100/ACT, 110/REST, 111/cycles)
	LED PB1/green		-> fast flashing

During TRAINING
===============
	T1					-> (nothing yet)
	T2					-> Stop training and go in "waiting mode", see main loop
	T3					-> Pause training

	LED PD0..4/red		-> show progress in current phase, current step flashes, always 1-5 leds
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