from pynput import keyboard
import serial

ser = serial.Serial('/dev/cu.usbmodem145101', 57600)

def on_press(key):
	if (key == 119):
		ser.writeln('w')
	elif (key == 97):
		ser.writeln('a')
	elif (key == 115):
		ser.writeln('s')
	elif (key == 100):
		ser.writeln('d')

def on_release(key):
	ser.writeln('98')

with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
	listener.join()
