import can
import usb


import sys
import time
import numpy as np


dev = usb.core.find(idVendor=0x1D50, idProduct=0x606F)


filters = [
    {"can_id": 0x451, "can_mask": 0x7FF, "extended": False},
    {"can_id": 0x00010203, "can_mask": 0x1FFFFFFF, "extended": True},
]
bus = can.Bus(interface="gs_usb", channel=dev.product, index=0, bitrate=250000, can_filters=filters)


# Receive a CAN message
while 1:
    try:
        
        received_message = bus.recv(100)
        if received_message:
            Ch1=np.int16((np.uint16(received_message.data[0]<<8)+np.uint16(received_message.data[1])))
            Ch2=np.int16((np.uint16(received_message.data[2]<<8)+np.uint16(received_message.data[3])))
            Ch3=np.int16((np.uint16(received_message.data[4]<<8)+np.uint16(received_message.data[5])))
            Ch4=np.int16((np.uint16(received_message.data[6]<<8)+np.uint16(received_message.data[7])))
            print(Ch1,
                  Ch2,
                  Ch3,
                  Ch4)
                 # (received_message.data[6]),received_message.data[7])

           
            #time.sleep(0.2)
    except Exception as e:
        print(f"Fehler beim Empfang der Nachricht: {e}")
        time.sleep(1) # Längere Pause nach einem Fehler
