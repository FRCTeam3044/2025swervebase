import serial.tools.list_ports

for pinfo in serial.tools.list_ports.comports():
    print(pinfo)