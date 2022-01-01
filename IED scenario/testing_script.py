import multiprocessing as mp
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import socket
import time

a = str(0.1) # Set max acceleration value for UR16e
v=str(0.1)   # Set max Velocity value of UR16e
HOST = "192.168.1.3" # The remote host
PORT = 30002 # The same port as used by the server
p_home = [0.2,-0.23,0.5]            # x , y , z coordinates of end effector relative to base frame at home position
r_home = [0,1.57,0]
p1 = p_home
row1 = str(r_home[0])
pitch1 = str(r_home[1])
yaw1 = str(r_home[2])
print ("Starting Program")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
time.sleep(0.5)
print ("Set output 1 and 2 high")
string1=("set_digital_out(1,True)" + "\n")
s.send (string1.encode())
time.sleep(1)
string2=("set_digital_out(2,True)" + "\n")
s.send (string2.encode())
time.sleep(1)
print("Starting slew to home position")
string6=('movel(p[' + str(p_home[0]) +',' + str(p_home[1]) + ',' + str(p_home[2]) + ',' + row1 + ',' + pitch1 +',' + yaw1 + '],a=' + a + ',v=' + v + ')\n')
s.send (string6.encode())
time.sleep(4)

