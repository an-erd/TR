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
- all led OFF / ON -> nicht mit Schleife, sondern als BIT-Operation
- FOR Schleife statt i++ durch DO WHILE (--i) ersetzen
- Board: Header für scl/sda sowie 6-poliger ISP durch PADS oder vias (und dann von der Rückseite kommend) ersetzen
