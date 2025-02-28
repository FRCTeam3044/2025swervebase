import serial
import time
import ntcore
import base64
import json

def connect_arduino():
    while True:
        try:
            arduino = serial.Serial(port='/dev/cu.usbmodem11401', baudrate=115200, timeout=1)
            print("Connected to Arduino")
            return arduino
        except serial.SerialException:
            print("Error: Could not find serial port. Retrying...")
            time.sleep(0.25)

def write_and_read(arduino, message):
    if not arduino.is_open:
        try:
            arduino.open()
        except serial.SerialException:
            print("Error: Could not open serial port")
            arduino = connect_arduino()
            if not arduino:
                return None
    try:
        print(f"Sending message: {message}")
        arduino.write(bytes(message + '\n', 'utf-8'))
        print("Message sent successfully")
        time.sleep(0.1)  # Give some time for Arduino to respond
        response = arduino.readline().decode('utf-8').strip()
        # print(f"Received response: {response}")
        # return response
        return
    except serial.SerialException:
        print("Error: Failed to write to serial port")
        arduino = connect_arduino()
        if arduino:
            try:
                arduino.write(bytes(message + '\n', 'utf-8'))
                print("Message sent successfully after reconnecting")
                # time.sleep(0.1)  # Give some time for Arduino to respond
                response = arduino.readline().decode('utf-8').strip()
                print(f"Received response: {response}")
                return response
            except serial.SerialException:
                print("Error: Failed to write to serial port after reconnecting")
                return None

def connect_network_tables():
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("Button Board LED Controller Companion")
    inst.setServer("localhost")
    table = inst.getTable("SmartDashboard/ButtonBoard")
    print("Connected to Network Tables")
    return table

arduino = connect_arduino()
table = connect_network_tables()

def callback(key, value, isNew):
    global data_to_send
    data_to_send = value

data_to_send = {}

while True:
    if arduino is None or not arduino.is_open:
        arduino = connect_arduino()
    data_to_send.update({"AlgaeMode": table.getBoolean("AlgaeMode", False)})
    data_to_send.update({"ClimbDown": table.getBoolean("ClimbDown", False)})
    data_to_send.update({"ClimbUp": table.getBoolean("ClimbUp", False)})
    encoded = base64.b64encode(json.dumps(data_to_send).encode('utf-8')).decode('utf-8')
    print(f"Encoded data: {encoded}")
    response = write_and_read(arduino, encoded)
    if response:
        print(f"Arduino response: {response}")
    data_to_send = {}
    # time.sleep(0.1)