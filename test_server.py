#!/usr/bin/python

import socket

# Listen on any address
UDP_IP = ""
UDP_SEND_IP = "192.168.250.228"
# On port 
UDP_PORT = 5005
client_address = (UDP_SEND_IP, UDP_PORT)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

while True:
    print "Sending message to %s" % (client_address, )
    sent = sock.sendto("Hello from Server", client_address)
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    print "received message: %s from %s" % (data, addr, )
