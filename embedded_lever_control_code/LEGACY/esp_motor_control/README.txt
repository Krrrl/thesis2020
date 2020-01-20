This folder contains the code running on the ESP8266MOD module that controlls the stepper motor and reads the encoder of the lever-task environment.

The lever is expected to be in the upright position when powering up the ESP.

The ESP module is controlled using serial connection.

The functionality of the module is as follows:
-Initialize lever to endpoint position
-Initialize lever to a random position
-set lever-position goal
-get current lever-position goal
-get lever-position
-get lever-distance to goal
-get reward for the current state
-get env ready state

The module has 4 different modes of operation:
-INIT
	Doing nothing other than listening for serial commands and reading encoder.

-READY
	Environment initialized, ready to begin next trail. Awaiting serial commands.

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
		resetEnvAbs	6
		resetEnvRan	7
		envReady	8
		startTrail	9

NOTE: It is important that the serial input does NOT have line-ending appended to its serial transmissions. Choose the option "no line ending" in either Arduino Serial Monitor or Termite terminal(or whatever other serial monitor you are using).


ENCODER:
The magnetic encoder has a range of approximately 23 steps over the feasible angles of the lever.





