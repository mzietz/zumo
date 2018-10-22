#!/usr/bin/env python


import bluetooth
import time

target_name = "HC06"
target_address = "00:21:13:00:2F:7E"
sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )

nearby_devices = bluetooth.discover_devices()

for bdaddr in nearby_devices:
    if target_name == bluetooth.lookup_name( bdaddr ):
        target_address = bdaddr
        break

if target_address is not None:
    print("found target bluetooth device with address "), target_address
else:
    print("could not find target bluetooth device nearby")

port = 1
try:
	sock.connect((target_address, port))
except:
	print("Failed to connect to"), target_address

sock.send("0")
time.sleep(3)
sock.send("1")
time.sleep(3)
sock.send("0")
time.sleep(3)
sock.send("1")
time.sleep(3)
print("Said Hello to "), target_address
