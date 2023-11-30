import time
import math
import serial

def txrx(ser, cmd, out_data , in_data):
   req = bytearray(64)
   resp = [0]*64
   req[0]=cmd
   req[1]=in_data[0]
   req[2]=in_data[1]
   ser.write(req)
   print(ser.in_waiting)
   for i in range(0, len(resp)   ):
        resp[i] =  ser.read()
   print(resp)

#   while(i<65):
#       print(f'{i}{ser.read()}')
#       i+=1
    


def main():
   ser = serial.Serial("COM5", 115200,timeout=15)
   out_data = []
   for i in range(0,2):
      #txrx(ser,1,out_data, [0,0] )
      txrx(ser,1,out_data, [0,0] )


if __name__ == '__main__':
    main()


