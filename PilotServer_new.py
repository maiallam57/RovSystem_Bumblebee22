#kill -9 $(ps -A | grep python | awk '{print $1}')
#sudo chmod 666 /dev/ttyACM0
import sys
sys.path.append('/home/rov/.local/lib/python3.6/site-packages')
import os
import depthai as dai
import subprocess
import sys
import pygame
from pygame.locals import *
import sensors
import socket
import time
import threading
from pymavlink import mavutil
from commands import *



class Server(threading.Thread):

    HOST =  '192.168.33.1' # socket.gethostname() #'192.168.1.2'  #

    PORT = 4072



    def __init__(self):



        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)

        self.s.bind((self.HOST,self.PORT))

        self.s.listen(1)

        # self.connect_client()

        

        self.t = threading.Thread(target=self.connect_client)

        self.t.start()
        # self.s.close() # this is how you close the socket



    def connect_client(self):

      

        c, address = self.s.accept()

        print(f"connection from {address} has been established")

        # self.snd_msg(c)
        
        self.thread_sensors=threading.Thread(target=self.snd_msg,args=(c,))
        self.thread_sensors.start()
        self.recv_msg(c)
        

    def recv_msg(self, c):   



        while True:
            # time.sleep(0.01)
            try:
                msg = c.recv(75)
                
            except:
                print("except")
                f=0
                # self.s.close()
                os._exit(0)

            
            print(msg.decode('utf-8'))

            recv_msg(msg.decode('utf-8'))



            if len(msg) < 1:

                break
         
        #kill -9 $(ps -A | grep python | awk '{print $1}')
        self.connect_client()


        

   

    def snd_msg(self, c):

        while True:

            try:
                # print(master.recv_match(type='AHRS2', blocking=True).to_dict())
                sensors=master.recv_match(type='SCALED_IMU').to_dict()
                # print(x)
                time.sleep(0.1)
                # print(master.recv_match(type='SCALED_PRESSURE').to_dict()['press_abs'])#.to_dict()['press_abs']
                sensors["pressure"]=master.recv_match(type='SCALED_PRESSURE2').to_dict()['press_abs']
                time.sleep(0.1)
                c.send(str(sensors).encode('utf-8'))
            except:
                pass

        


def heart_beats():



        while True:

            print("heartbeat")

            

            master.mav.heartbeat_send(

                mavutil.mavlink.MAV_TYPE_GCS,

                mavutil.mavlink.MAV_AUTOPILOT_INVALID,

                0, 0, 0)

            time.sleep(1)

try:

    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

except:

    master = mavutil.mavlink_connection("/dev/ttyACM1", baud=115200)

heart_beat=threading.Thread(target=heart_beats)

heart_beat.start()
sensors.init_sensors(master)

def recv_msg (msg):

    

    rc_channels=[0,0,0,0,0,0,0,0,0]

    if (msg[0:2]=="rc"):

        start=2

        for i in range(9):

            rc_channels[i]=int(msg[start:start+4])

            start+=4

        ROV.set_RC_channel_pwm(rc_channels)

    # elif (msg[0:2]=="se"):

    #     ROV.set_servo_pwm(int(msg[2:6]))

    elif (msg[0:2]=="og"):

        ROV.open_gripper1()

    elif (msg[0:2]=="cg"):

        ROV.close_gripper1()
    elif (msg[0:2]=="o2"):

        ROV.open_gripper2()

    elif (msg[0:2]=="c2"):

        ROV.close_gripper2()

    elif (msg[0:2]=="ar"):

        ROV.arming()

    elif (msg[0:2]=="dr"):

        ROV.disarming()

    elif (msg[0:2]=="in"):

        ROV.init_thrusters()

    elif (msg[0:2]=="lf"):

        ROV.led_off()

    elif (msg[0:2]=="ln"):

        ROV.led_on()   

    elif (msg[0:2]=='st'):       #must select a specific mode 
        ROV.flight_Mode('STABILIZE')

    elif (msg[0:2]=='ah'):    #this mode need a signal 
        ROV.flight_Mode('ALT_HOLD')

    elif (msg[0:2]=='mn'):       #must select a specific mode 
        ROV.flight_Mode('MANUAL')


     
serverO=Server(master)



ROV=commands(master)

# subprocess.Popen('python3 colorCameraq_stream_script.py',shell=True)



#ROV.arming()
