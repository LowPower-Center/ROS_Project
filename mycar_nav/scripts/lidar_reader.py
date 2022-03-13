from time import sleep
from serial import Serial
import numpy as np
from math import fabs, radians

_MAX_DISTANCE_MM = 10000 # Maximum distance in mm   

# Serial settings
_COM_PORT = "/dev/lidar"
_BAUDRATE = 115200
_SERIAL_TIMEOUT = 1
_READ_BATCH = 7944 # 360+1 degrees * 22 bytes per package

ser = Serial(_COM_PORT, baudrate=_BAUDRATE, timeout=_SERIAL_TIMEOUT) 

class LidarReader():
    def __init__(self, serial: Serial):
        self.serial = serial
        self.work_state = 0
        self.cache = [_MAX_DISTANCE_MM/1000]*90
        self.intensities = [0]*90
        self.speed = 0


    def start(self):
        sleep(0.25)
        self.send_lidar_init(1)
    def stop(self):
        sleep(0.25)
        self.send_lidar_init(0)

    def send_lidar_init(self,flag):
        if flag:
            self.serial.write(b'startlds$')
        else: 
            self.serial.write(b'stoplds$')
        return True



    def wait_read(self):
        packages = 0
        while packages == 0:
            data = self.serial.read(_READ_BATCH)
            readed = len(data)
            i = 0
            while i < readed:
                if data[i] != 0xFA and (data[i] != 0x5A or self.work_state == 2):
                    i+=1
                    continue
                self.work_state = 1 if data[i] == 0x5A else 2
                msgsize = 3 if self.work_state == 1 else 21
                message = data[i+1:i+1+msgsize]
                check_list=data[i:i+1+msgsize]
#                print("")
#                print(check_list.hex(' '))
#                print(self.check_checksum(check_list))
#                print("")
                i+=msgsize
                if (len(message) < msgsize):
                    print(len(message), msgsize, i)
                    continue
                packages += 1
                if self.work_state == 2 and self.check_checksum(check_list):
                    self.parse_message(message)
                else:
                    self.parse_init_message(message)
            
        return self.work_state

    def parse_message(self, packet: bytes):
        endian = 'little'
        angle = packet[0]
   
        speed = int.from_bytes((packet[1:3]), endian)
        speed=(speed>>6)+1
        
        if angle == 0xFB :
            # raise Exception("Speed error!")
            print("Speed error! ", speed)
            print(packet.hex(' '))

            return
        else:
            index=angle-0xA0
            angle = (angle-0xA0) * 4
        # Parsing distances
        
        if index>89:
            return
        intensities = []
        distances = []
        for i in range(4):
            offset = 3 + i*4
            distance = int.from_bytes((packet[offset:offset+2]), endian, signed=False)
            distance = distance & 0x3fff
            intensity = int.from_bytes((packet[offset+2:offset+4]), endian, signed=False)
            if distance > _MAX_DISTANCE_MM:
                distance=_MAX_DISTANCE_MM
            if intensity != 0:
                intensities.append(intensity)
                distances.append(distance)
        distance = np.median(distances)/1000
        intensity = min(2000, np.median(intensities))
        self.intensities[index]=intensity
        self.cache[index]=distance

            #print("Speed ", speed, "; angle ", angle+i, "; distance", distance, "; reflection", reflect)


        #print("Speed ", speed, "; angle ", angle, "; distance", distance, "; reflection", reflect)

        self.speed = speed

    def parse_init_message(self, packet):
        distance = int.from_bytes((packet[2:4]), 'big')
        print("Init, distance ", distance)



    def check_checksum(self, packet:bytes):

        checksum = int.from_bytes(packet[-2:], 'little')
        check32_mask=0
        for i in range(20):
            check32_mask+=int.from_bytes(packet[i:i+1], 'little')

        if (checksum != check32_mask):
            print("Checksum error!")
            return False
        return True

