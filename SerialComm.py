import time
import serial
import globals

class SerialComm:
    def __init__(self,port,baudrate,timeout=5):
        self.ser = serial.Serial(port, baudrate,timeout=timeout)
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
    
    def __del__(self):
        self.ser.close()

    def set_adress(self, adress):
        self.adress = adress

    def send_trajectory(self, coords):
        self.ser.timeout = 100
        value = self.write_file(coords, 1)
        self.ser.timeout = self.timeout
        return value

    def goto(self, coords):
        return self.write_file(coords, 0)

    def write_file(self, coords, trajectory_flag = 0):
        function_code = 0x15
        int_coords = [int(coord) for submatrix in coords for coord in submatrix]

        message = '' + globals.start_of_message                     # Byte 0
        message += self.encode(self.adress)                       # Bytes 1 e 2
        message += self.encode(function_code)                     # Bytes 3 e 4
        message += self.encode(trajectory_flag)                        # Bytes 5 e 6
        message += self.encode(len(int_coords))                       # Bytes 7 e 8
        for coord in int_coords:
            print(f"Coord: {coord}")
            message += self.encode((coord & 0xff00) >> 8) + self.encode(coord & 0xff)
        message += self.encode(self.calculate_lrc(message[1:])) 
        message += globals.end_of_message   

        print(f"Sending message: {message}fim")
        self.send_message(message)
        response = self.receive_response()
        return self.decode(response[9], response[10])

    def read_register(self, register_value):
        function_code = 0x03

        message = '' + globals.start_of_message                     # Byte 0
        message += self.encode(self.adress)                       # Bytes 1 e 2
        message += self.encode(function_code)                     # Bytes 3 e 4
        message += self.encode(1)                                 # Bytes 5 e 6
        message += self.encode(register_value)                    # Bytes 7 e 8
        message += self.encode(self.calculate_lrc(message[1:])) # Bytes 9 e 10
        message += globals.end_of_message                           # Bytes 11 e 12
        print(f"Sending message: {message}")
        self.send_message(message)
        response = self.receive_response()
        return self.decode(response[7], response[8])

    def write_register(self, register_value, value_to_write):
        function_code = 0x06

        message = '' + globals.start_of_message                     # Byte 0
        message += self.encode(self.adress)                       # Bytes 1 e 2
        message += self.encode(function_code)                     # Bytes 3 e 4
        message += self.encode(1)                                 # Bytes 5 e 6
        message += self.encode(register_value)                    # Bytes 7 e 8
        message += self.encode(value_to_write)                    # Bytes 9 e 10
        message += self.encode(self.calculate_lrc(message[1:])) # Bytes 10 e 11
        message += globals.end_of_message                           # Bytes 11 e 12
        print(f"Sending message: {message}")
        self.send_message(message)
        response = self.receive_response()
        return response

    def encode(self, value):
        x = value & 0x0f
        y = (value & 0xf0) >> 4
        return hex(y)[-1] + hex(x)[-1]

    def decode(self, value_high, value_low):
        x=int(value_high,16)
        y=int(value_low,16)
        return (x<<4) | y
    
    def send_message(self, message):
        self.ser.write(message.encode())

    def receive_response(self):
        response = self.ser.read_until(expected=globals.end_of_message.encode(), size=30)
        response = response.decode()[:-2]
        print("Response: ", response)
        if not self.check_lrc(response[-2:], response[1:-2]):
            print("Error in LRC")
            return None
        return response

    def calculate_lrc(self, frame):
        sum = 0
        for i in range(len(frame)):
            sum += ord(frame[i])
        sum = (0xff - (sum & 0xff)) + 1
        return sum
    
    def check_lrc(self, received_lrc, frame):
        received_lrc = self.decode(received_lrc[0],received_lrc[1])
        lrc = self.calculate_lrc(frame)
        return lrc == received_lrc
    


    
