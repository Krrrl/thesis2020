This folder contains the arduino code for the ustepper module, for the lever manipulation task.
A unit test of the module can be conducted by running the interface.py script on its own.


The functionality of the module is as follows:
-Initialize lever to endpoint position
-Initialize lever to a random position
-set lever-position goal
-get current lever-position goal
-get lever-position
-get lever-distance to goal
-get reward for the current state
-get env ready state
-start next trail
-reset lever to endpoint
-reset lever to random point
-reset lever to midpoint

The module has 4 different modes of operation:
-INIT
	Doing nothing other than listening for serial commands and reading encoder.

-READY
	Environment initialized, ready to begin next trail. Listening on serial.

-Trail running
	Motor disenganged, encoder read, listening on serial.

-Trail ended
	motor engaged, encoder read, listening on serial.
	

The possible commands are:

	operation:		encoding:
		setGoal		1
		getGoal		2
		getAbsPos	3
		getRelPos	4
		getReward	5
		resetEnvABS	6
		resetEnvRAN	7
		envReady	8
		startTrail	9
		resetEnvMID 	10

NB! Please do NOT access these commands directly. Use the provided PySerial interface, located in the embedded_lever_interface from the manipulator_training package.
