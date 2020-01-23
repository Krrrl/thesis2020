This module contains the Lever class, which holds the serial interface to the ustepper lever control module.

The module is used by the ManipulatorLever-gym environment, so you dont need to make use of it directly.



ISSUES:
Make sure that the U2D2 serial converter(OpenManipulator) is connected to the computer BEFORE you connect the lever.
If not, the lever will take the TTY0 serial port, and this code is not flexible to accommodate that.
Its a really ugly feature.



COMMENTS TO THE CODE:
The code is very simple, utilizing the pyserial native library.

Note how things that are written to serial using this interface has to be of the type STRING, and has to be encoded.
Make sure you use the write_to_serial()-function if you are implementing your own commands, or include the aforementioned in your code to avoid trouble.
