#!/usr/bin/env python

import sys

import serial

#ser = serial.Serial();

class Lever(object):
	
	def __init__(self):
		self.serial_commands = {"setGoal" 		: 1, 
								"getGoal" 		: 2,
								"getAbsPos"		: 3, 
								"getRelPos"		: 4, 
								"getReward"		: 5, 
								"resetEnvABS" 	: 6, 
								"resetEnvRAN" 	: 7,
								"envReady"		: 8,
								"startTrail"	: 9
								}
		self.init_serial_connection()


	def init_serial_connection(self):
		self.ser = serial.Serial(
			port = "/dev/ttyUSB1",
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
		print("Opened serial port: {}".format(self.ser.name))
		status = self.ser.isOpen()
		print("Serial port status: {}".format(status))
		

	def set_Goal(self, goal):
		self.write_to_serial(self.serial_commands.get("setGoal", 0))
		self.write_to_serial(goal)

	def get_Goal(self):
		received = None
		while(received == None):
			self.write_to_serial(self.serial_commands.get("getGoal", 0))
			received = self.read_data_from_serial()
		return received

	def get_ABS_Pos(self):
		received = None
		while(received == None):
			self.write_to_serial(self.serial_commands.get("getAbsPos", 0))
			received = self.read_data_from_serial()
		return received
		
	def get_REL_Pos(self):
		received = None
		while(received == None):
			self.write_to_serial(self.serial_commands.get("getRelPos", 0))
			received = self.read_data_from_serial()
		return received

	def get_Reward(self):
		received = None
		while(received == None):
			self.write_to_serial(self.serial_commands.get("getReward", 0))
			received = self.read_data_from_serial()
		return received

	def reset_ABS(self):
		self.write_to_serial(self.serial_commands.get("resetEnvABS", 0))

	def reset_RAN(self):
		self.write_to_serial(self.serial_commands.get("resetEnvRAN", 0))

	def env_Ready(self):
		received = None
		while(received == None):
			self.write_to_serial(self.serial_commands.get("envReady", 0))
			received = self.read_data_from_serial()
		return received

	def start_Trail(self):
		received = None
		while(received == None):
			self.write_to_serial(self.serial_commands.get("startTrail", 0))
			received = self.read_data_from_serial()
		return received

	def parse_data_from_serial(self, data):
		if(data):
			parsed_data = int(data)
			return parsed_data
		else:
			print("RECEIVED EMPTY STRING TO PARSE! Trying again")
			return None

	def read_data_from_serial(self):		
		#BLOCKING (cause no timeout is set on read_until)
		data = self.ser.readline()
		parsed_data = self.parse_data_from_serial(data)
		return parsed_data

	def write_to_serial(self, data):
		self.ser.write(str(data).encode())
		self.ser.write(str('\n').encode())

	def keyboard_input(self):
		print("\n")
		print("What would you like to do? ")
		command = raw_input()
		return int(command)

if __name__ == '__main__':
	lever = Lever()
	
	print("Port status! ")
	print(lever.ser.isOpen())

	while(1):
		command = lever.keyboard_input()
		if(command == 1):
			lever.set_Goal(-100)
		elif(command == 2):
			print("\n")
			print(lever.get_Goal())
		elif(command == 3):
			print("\n")
			print(lever.get_ABS_Pos())
		elif(command == 4):
			print("\n")
			print(lever.get_REL_Pos())
		elif(command == 5):
			print("\n")
			print(lever.get_Reward())
		elif(command == 6):
			print("Sending reset_ABS signal!")
			lever.reset_ABS()
		elif(command == 7):
			print("Sending reset_RAN signal!")
			lever.reset_RAN()
		elif(command == 8):
			print("\n")
			print(lever.env_Ready())
		elif(command == 9):
			print("\n")
			print(lever.start_Trail())
		else:
			print("Not a valid command.")
