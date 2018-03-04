#!/usr/bin/env python
__author__ = 'Administrator'
import serial

class mySerial:
    def __init__(self):
        self.ser = serial.Serial()

    def openSerial(self, serial_port='/dev/ttyUSB1',serial_baudrate=115200):
        self.ser.baudrate = serial_baudrate
        #self.ser.port = "COM"+str(serialNum)
        self.ser.port = serial_port
        self.ser.timeout =1
        self.ser.open()

    def closeSerial(self):
        self.ser.close()

    def readVal(self):
        line = self.ser.readline()
        return line
    def sendVal(self, data):
        self.ser.write(data)


if __name__ == '__main__':
    ser = mySerial()
    ser.openSerial()
    for i in range(1,10000):
        print ser.readVal()

    ser.closeSerial()

'''
#!/usr/bin/env python
# -*- coding: utf-8 -*
 
import serial
import serial.tools.list_ports
 
port_list = list(serial.tools.list_ports.comports())
 
if len(port_list) <= 0:
    print "The Serial port can't find!"
     
else:
    port_list_0 =list(port_list[0])
 
    port_serial = port_list_0[0]
 
    ser = serial.Serial(port_serial,9600,timeout = 60)
 
    print "check which port was really used >",ser.name
'''
