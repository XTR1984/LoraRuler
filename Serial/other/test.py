import time
import math
import serial


def main():
   ser = serial.Serial("COM10", 9600,timeout=5)

   req = bytearray(64)
   resp = [0]*64
   req[0]=0x14
   req[1]=0
   req[2]=0
   ser.write(req)
   for i in range(0, len(resp)   ):
        resp[i] =  ser.read()
   print(resp)

if __name__ == '__main__':
    main()


