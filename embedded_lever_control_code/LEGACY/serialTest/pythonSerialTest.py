import serial

from lever_interface.interface import Lever

class serialReader():

	def __init__(self):
		print("I'm up!")

	def init_serial(self):
		self.ser = serial.Serial(
				port = "/dev/ttyUSB0",
				baudrate = 115200,
				bytesize = serial.EIGHTBITS, 
				parity = serial.PARITY_NONE,
				stopbits = serial.STOPBITS_ONE, 
				timeout = 1,
				xonxoff = False,
				rtscts = False,
				dsrdtr = False,
				writeTimeout = 2
			)
		print("Opening the serial port: {}".format(self.ser.name))
		print("The port status is: {}".format(self.ser.isOpen()))

	def read_serial_data(self):
		data = self.ser.readline()
		if(data):
			print("I received some data!")
			return data
		else:
			print("Aint heard nothing..")
			return None


class divClass():
	
	def __init__(self, serReader):
		self.mySer = serReader
		print("Here i go initializing")
		self.mySer.init_serial()

	def loopy(self):
		print("Watch me looooop")
		print("Watch me priiint: ")
		print(self.mySer.read_serial_data())


if __name__ == '__main__':
	reader = serialReader()

	diver = divClass(reader)


	while True:
		received = diver.loopy()
		print(received)
