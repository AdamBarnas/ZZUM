import serial
import time
import numpy as np
import matplotlib.pyplot as plt

class GCodeSender:
    def __init__(self, port, baudrate=115200, timeout=1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # Wait for the connection to establish

    def send_gcode(self, gcode):
        self.ser.write((gcode + '\n').encode())
        response = self.ser.readline().decode().strip()
        return response

    def close(self):
        self.ser.close()

def test_serial_connection(port):
    sender = GCodeSender(port)
    response = sender.send_gcode('M105')  # Request temperature
    print(f"Response from printer: {response}")
    sender.close()

