import serial
import serial.tools.list_ports
import struct
import time



class serial_control(object):
    def __init__(self, port):
        print('Avaliable ports:')
        # Get a list of available serial ports
        available_ports = list(serial.tools.list_ports.comports())
        
        # Print the list of available ports
        for ports in available_ports:
            print(ports)

        # Replace 'COM1' with the appropriate port name (e.g., '/dev/ttyUSB0' on Linux)
        self.ser = serial.Serial(port, baudrate=115200, timeout=1)

    def read(self):
        # Read a specific number of bytes
        #data = ser.read(10)

        # Read a line of text (until a newline character)
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8')

        else:
            return 'No message.'

        return str(line)

    def write(self,data):
        self.ser.write(data)


    def close(self):
        self.ser.close()

    def conv(self,float_value):
        float_bytes = struct.pack('<f', float_value) # Convert float to bytes in big-endian format
        return float_bytes

    def goto(self, joint0, joint1, joint2, joint3):
        bytes_goto = b'\x02'

        bytes_joint0 = self.conv(float(joint0))
        bytes_joint1 = self.conv(float(joint1))
        bytes_joint2 = self.conv(float(joint2))
        bytes_joint3 = self.conv(float(joint3))

        #float_bytes = bytes_goto + bytes_joint0 + bytes_joint1 + bytes_joint2 + bytes_joint3
        #hex_representation = ' '.join(hex(byte)[2:].zfill(2) for byte in float_bytes)
        #print(hex_representation)

        self.write(bytes_goto + bytes_joint0 + bytes_joint1 + bytes_joint2 + bytes_joint3)

    def go(self, joint0, joint1, joint2, joint3):
        bytes_go = b'\x08'

        bytes_joint0 = self.conv(float(joint0))
        bytes_joint1 = self.conv(float(joint1))
        bytes_joint2 = self.conv(float(joint2))
        bytes_joint3 = self.conv(float(joint3))

        self.write(bytes_go + bytes_joint0 + bytes_joint1 + bytes_joint2 + bytes_joint3)

    def kinematics(self, X, Y, Z):
        bytes_kinematics = b'\x04'

        bytes_x = self.conv(float(X))
        bytes_y = self.conv(float(Y))
        bytes_z = self.conv(float(Z))

        self.write(bytes_kinematics + bytes_x + bytes_y + bytes_z)

    def forward_kinematics(self):
        self.write(b'\x06') #TODO: check the index for forward kinematics
        time.sleep(0.1)
        coordinates = self.read()
        px = coordinates.index('X:')
        py = coordinates.index('Y:')
        pz = coordinates.index('Z:')
        X = coordinates[px+2:py-1]
        Y = coordinates[py+2:pz-1]
        Z = coordinates[pz+2: ]

        return float(X), float(Y), float(Z)

    def kinematics_top(self, X, Y, Z):
        bytes_kinematics = b'\x10'

        bytes_x = self.conv(float(X))
        bytes_y = self.conv(float(Y))
        bytes_z = self.conv(float(Z))

        self.write(bytes_kinematics + bytes_x + bytes_y + bytes_z)
        
    def emergencystop(self):
        self.write(b'\xFE')

    def stop(self):
        self.write(b'\xFF')

    def home(self):
        self.write(b'\x00')