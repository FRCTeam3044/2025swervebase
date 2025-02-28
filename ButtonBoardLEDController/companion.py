import serial
import time
import ntcore
import base64
import json

def connect_arduino(port):
    while True:
        try:
            arduino = serial.Serial(port, baudrate=115200, timeout=1)
            print("Connected to Arduino")
            return arduino
        except serial.SerialException:
            print("Error: Could not find serial port. Retrying...")
            time.sleep(0.1)

def write(arduino, message):
    if not arduino.is_open:
        try:
            arduino.open()
        except serial.SerialException:
            print("Error: Could not open serial port")
            arduino = connect_arduino()
            if not arduino:
                return None
    try:
        arduino.write(bytes(message + '\n', 'utf-8'))
        return
    except serial.SerialException:
        print("Error: Failed to write to serial port")
        arduino.close()
        arduino = connect_arduino()
        if arduino:
            try:
                arduino.write(bytes(message + '\n', 'utf-8'))
                print("Message sent successfully after reconnecting")
                return
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

arduino1 = connect_arduino("/dev/cu.usbmodem1401")
arduino2 = connect_arduino("/dev/cu.usbmodem1401")
arduino3 = connect_arduino("/dev/cu.usbmodem1401")

table = connect_network_tables()

def callback(key, value, isNew):
    global data_to_send
    data_to_send = value

data_to_send = []

while True:
    # if arduino1 is None or not arduino1.is_open:
    #     arduino1 = connect_arduino()
    # if arduino2 is None or not arduino1.is_open:
    #     arduino2 = connect_arduino()
    # if arduino3 is None or not arduino1.is_open:
    #     arduino3 = connect_arduino()

# Board A
    def check_location():
        location = table.getString("CoralReefLocation", "A")
        letters = "ABCDEFGHIJKL"
        data_to_send = [True if location == letter else False for letter in letters]
        return data_to_send
    data_to_send = check_location()
# Board B
    data_to_send.append((table.getString("IntakeStation", "L1") == "L1"))
    data_to_send.append((table.getString("IntakeStation", "L1") == "L2"))
    data_to_send.append((table.getString("IntakeStation", "L1") == "L3"))

    data_to_send.append(table.getBoolean("AlgaeMode", False))

    data_to_send.append((table.getString("IntakeStation", "L1") == "R1"))
    data_to_send.append((table.getString("IntakeStation", "L1") == "R2"))
    data_to_send.append((table.getString("IntakeStation", "L1") == "R3"))
    data_to_send.append(False)
    data_to_send.append(False)
    data_to_send.append(False)
    data_to_send.append(False)
    data_to_send.append(False)
# Board C
    data_to_send.append(False)
    data_to_send.append(False)
    data_to_send.append(False)
    data_to_send.append(False)

    # climbers
    data_to_send.append(False)
    data_to_send.append(False)

    # ReefLevel
    data_to_send.append(table.getString("CoralReefLevel", "4") == "4")
    data_to_send.append(table.getString("CoralReefLevel", "4") == "3")
    data_to_send.append(table.getString("CoralReefLevel", "4") == "2")
    data_to_send.append(table.getString("CoralReefLevel", "4") == "1")

    # Net/Processor
    data_to_send.append(False)
    data_to_send.append(False)

    encoded = base64.b64encode(json.dumps(data_to_send).encode('utf-8')).decode('utf-8')
    # print(f"Encoded data: {encoded}")
    write(arduino1, encoded)
    # write(arduino2, encoded)
    # write(arduino3, encoded)

    data_to_send = []