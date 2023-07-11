import sys
sys.path.append('/home/rov/.local/lib/python3.6/site-packages')
from doctest import master

import threading
import pygame
from pymavlink import mavutil
import sys
import Jetson.GPIO as GPIO      # GPIO library
import serial
import time

# import ServerOOP



# serverO = ServerOOP.Server()
import serial.tools.list_ports

portData=serial.tools.list_ports.comports()

print(portData[0])

arduino = serial.Serial(port="/dev/ttyUSB0", baudrate=9600)

arduino.timeout =0.1

class commands:

    

    def __init__(self,master):

        self.master=master

        self.master.arducopter_disarm()

        self.Gripper_pin1 = 29  
        self.Gripper_pin2 = 35
        self.led_pin1=31


        GPIO.setmode(GPIO.BOARD) 
        GPIO.setup(self.Gripper_pin1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.Gripper_pin2, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.led_pin1, GPIO.OUT, initial=GPIO.LOW)


    def arming(self):

        

        self.master.mav.command_long_send(

            self.master.target_system,

            self.master.target_component,

            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,

            0,

            1, 0, 0, 0, 0, 0, 0)

            

        self.master.motors_armed_wait()

        print("armed")

        self.init_thrusters()

        # self.set_servo_pwm()

        return True



    def disarming(self):



        self.master.mav.command_long_send(

            self.master.target_system,

            self.master.target_component,

            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,

            0,

            0, 0, 0, 0, 0, 0, 0)  

        self.master.motors_disarmed_wait()

        print ("disarmed")

        return True

    def set_RC_channel_pwm (self,rc_channel_values):


        print (rc_channel_values)

        self.master.mav.rc_channels_override_send(

        self.master.target_system,                # target_system

        self.master.target_component,             # target_component

        *rc_channel_values)                  # RC channel list, in microseconds.



        

    def set_relay(self, relayNO=1, state=0):

    

    # self.master.set_servo(servo_n+8, microseconds) or:

        self.master.mav.command_long_send(

        self.master.target_system, self.master.target_component,

        mavutil.mavlink.MAV_CMD_DO_SET_RELAY,

        0,            # first transmission of this command

        relayNO ,  # servo instance, offset by 8 MAIN outputs

        state, # PWM pulse-width

        0,0,0,0,0     # unused parameters

    )



    

    

    def active_high(self, pin):
 

        GPIO.output(pin, GPIO.HIGH) 


    def active_low(self, pin):


        GPIO.output(pin, GPIO.LOW)

        



    #setting channels

    def pitch ( self,channel_values,PWM=1500 ):

        

        channel_values[0]=PWM

        self.set_RC_channel_pwm(channel_values)

        print(channel_values)



        return channel_values



    def roll ( self,channel_values,PWM=1500 ):


        channel_values[1]=PWM

        self.set_RC_channel_pwm(channel_values)

        print(channel_values)


        return channel_values

    

    def throttle ( self,channel_values,PWM=1500 ):



        channel_values[2]=PWM

        self.set_RC_channel_pwm(channel_values)

        print(channel_values)



        return channel_values



    def yaw ( self,channel_values,PWM=1500 ):



        channel_values[3]=PWM

        self.set_RC_channel_pwm(channel_values)

        print(channel_values)



        return channel_values



    def forward ( self,channel_values,PWM=1500 ):

        print("yes")



        channel_values[4]=PWM

        self.set_RC_channel_pwm(channel_values)

        print(channel_values)



        return channel_values



    def lateral ( self,channel_values,PWM=1500 ):



        channel_values[5]=PWM

        self.set_RC_channel_pwm(channel_values)

        print(channel_values)



        return channel_values






    def init_thrusters(self):

        

        channel_values=[1500 for _ in range(9)]

        self.set_RC_channel_pwm(channel_values)

        print(channel_values)



        return channel_values



    def open_gripper1 (self):
        #self.set_relay(0,1)
        #self.set_relay(1,1)
        #self.set_relay(2,1)
        #self.set_relay(3,1)
        #self.set_relay(4,1)
        #self.set_relay(5,1)
        #self.set_relay(6,1)
        print("open1")
        msg='o'
        arduino.write(msg.encode())
        time.sleep(0.1)
        print(arduino.readline().decode())
	
    def open_gripper2 (self):
        #self.set_relay(0,1)
        #self.set_relay(1,1)
        #self.set_relay(2,1)
        #self.set_relay(3,1)
        #self.set_relay(4,1)
        #self.set_relay(5,1)
        #self.set_relay(6,1)
        print("open2")
        msg='q'
        arduino.write(msg.encode())
        time.sleep(0.1)
        print(arduino.readline().decode())

    def close_gripper1 (self):
        #self.set_relay(0,0)
        #self.set_relay(1,0)
        #self.set_relay(2,0)
        #self.set_relay(3,0)
        #self.set_relay(4,0)
        #self.set_relay(5,0)
        #self.set_relay(6,0)
        print("close")
        msg='c'
        arduino.write(msg.encode())
        time.sleep(0.1)
        print(arduino.readline().decode())


    def close_gripper2 (self):
        #self.set_relay(0,0)
        #self.set_relay(1,0)
        #self.set_relay(2,0)
        #self.set_relay(3,0)
        #self.set_relay(4,0)
        #self.set_relay(5,0)
        #self.set_relay(6,0)
        print("close2")
        msg='z'
        arduino.write(msg.encode())
        time.sleep(0.1)
        print(arduino.readline().decode())

    def led_on (self):
        msg='n'
        arduino.write(msg.encode())
        time.sleep(0.1)
        print(arduino.readline().decode())

        # self.set_relay(1,1)
        # self.set_relay(2,1)
        # self.set_relay(3,1)
        # self.set_relay(4,1)
        # self.set_relay(5,1)
        # self.set_relay(6,1)
        
        # self.active_high(self.led_pin1)


    def led_off(self):

        msg='f'
        arduino.write(msg.encode())
        time.sleep(0.1)
        print(arduino.readline().decode())

        # self.active_low(self.led_pin1)


    


    def flight_Mode(self,mo):
        # Choose a mode
        mode = mo

        # Check if mode is available
        if mode not in master.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(master.mode_mapping().keys()))
            sys.exit(1)

        # Get mode ID
        mode_id = master.mode_mapping()[mode]
        # Set new mode
        # master.mav.command_long_send(
        #    master.target_system, master.target_component,
        #    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        #    0, mode_id, 0, 0, 0, 0, 0) or:
        # master.set_mode(mode_id) or:
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)

        while True:
            # Wait for ACK command
            ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
            ack_msg = ack_msg.to_dict()

            # Check if command in the same in `set_mode`
            if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
                continue

            # Print the ACK result !
            print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
            break

    


# msg="rc150015001500150015001500150015001500"



