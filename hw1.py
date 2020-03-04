import time
import numpy as np
import RPi.GPIO as gpio
import matplotlib.pyplot as plt
from picamera import PiCamera
from picamera.array import PiRGBArray
from gpiozero import PWMOutputDevice

# Motor A, Left Side GPIO CONSTANTS
PWM_FORWARD_LEFT_PIN = 18	# IN1 - Forward Drive
PWM_REVERSE_LEFT_PIN = 23	# IN2 - Reverse Drive
# Motor B, Right Side GPIO CONSTANTS
PWM_FORWARD_RIGHT_PIN = 24	# IN3 - Forward Drive
PWM_REVERSE_RIGHT_PIN = 25	# IN4 - Reverse Drive

# Initializing the motor pins and the speed
speed = 0.4
forwardLeft = PWMOutputDevice(PWM_FORWARD_LEFT_PIN, True, 0, 1000)
backLeft = PWMOutputDevice(PWM_REVERSE_LEFT_PIN, True, 0, 1000)
forwardRight = PWMOutputDevice(PWM_FORWARD_RIGHT_PIN, True, 0, 1000)
backRight = PWMOutputDevice(PWM_REVERSE_RIGHT_PIN, True, 0, 1000)

# Initializing the ultrasonic sensor
gpio.setmode(gpio.BCM)
trigger = 20
echo = 21
gpio.setup(trigger, gpio.OUT)
gpio.setup(echo, gpio.IN)

def stopDCMotor():
	forwardLeft.value = 0
	backLeft.value = 0
	forwardRight.value = 0
	backRight.value = 0
 
def goForward(speed):
	forwardLeft.value = speed
	backLeft.value = 0
	forwardRight.value = speed
	backRight.value = 0
 
def goBackward(speed):
	forwardLeft.value = 0
	backLeft.value = speed
	forwardRight.value = 0
	backRight.value = speed
    
def goLeft(rspeed,lspeed):
	forwardLeft.value = rspeed 
	backLeft.value = 0
	forwardRight.value = lspeed
	backRight.value = 0
 
def goRight(rspeed,lspeed):
	forwardLeft.value = rspeed
	backLeft.value = 0
	forwardRight.value = lspeed
	backRight.value = 0
    
def leftBack(rspeed,lspeed):
	forwardLeft.value = 0
	backLeft.value = rspeed
	forwardRight.value = 0
	backRight.value = lspeed
 
def rightBack(rspeed,lspeed):
	forwardLeft.value = 0
	backLeft.value = rspeed
	forwardRight.value = 0
	backRight.value = lspeed
 
    
def getDistance():
    gpio.output(trigger, 1)
    time.sleep(0.00001)
    gpio.output(trigger, 0)
    startTime = time.time()
    stopDCMotorTime = time.time()
    while gpio.input(echo) == 0:
        startTime = time.time()
    while gpio.input(echo) == 1:
        stopDCMotorTime = time.time()
    timeSpent = stopDCMotorTime - startTime
    distance = (timeSpent * 34300) / 2
    return distance
             
def motor_mission1():
    
    #forward for 2 seconds
    goForward(1)
    time.sleep(2)
    #wait for 1 seconds
    stopDCMotor()
    time.sleep(1)
    #turn right
    goRight(1,1);
    time.sleep(0.79)
    #go backward for 2 seconds
    goBackward(1)
    time.sleep(2)
    #wait for 1 seconds
    stopDCMotor()
    time.sleep(1)
    #turn left
    goRight(1,1)
    time.sleep(0.79)
    #stopDCMotor the car
    stopDCMotor()

def motor_mission2():
    
    #right rotate 360 degrees
    goRight(1,1)
    time.sleep(1.2)
    #wait for 1 seconds
    stopDCMotor()
    time.sleep(1)
    #left rotate 360 degrees
    goLeft(1,1)
    time.sleep(1.2)
    
def motor_mission3():
    #slowly go forward
    goForward(0.4)
    time.sleep(1)
    #slightly increase the speed
    goForward(0.6)
    time.sleep(1)
    #get the maximum speed
    goForward(1)
    time.sleep(1)
    stopDCMotor()
    
def motor_mission4():    
    #turn right forward
    goRight(1,0.6)
    time.sleep(1.25)
    #forward for 1 seconds
    goForward(1)
    time.sleep(1)
    #turn left forward    
    goLeft(0.6,1)
    time.sleep(1.6)
    #forward for 1 seconds
    goForward(1)
    time.sleep(1)
    stopDCMotor()
    
def motor_mission5():
    
    #turn right back
    rightBack(1,0.6)
    time.sleep(0.9)
    #backward for 1 seconds
    goBackward(1)
    time.sleep(1)
    #turn left back    
    leftBack(0.6,1)
    time.sleep(1.4)
    #backward for 1 seconds
    goBackward(1)
    time.sleep(1)
    stopDCMotor()
    
def sonic_mission1():
    while True:
        dist = getDistance()
        if(dist>30):
            goForward(speed)
        elif(dist<30):
            goBackward(speed)
        else:
            stopDCMotor()        
        
def sonic_mission2():
    turn = False
    while True:
        dist = distance()
        if(dist>30):
            forward(speed)
            if turn == True:
                time.sleep(2)
                stopDCMotor()
                break
        else:
            reverse(speed)
            time.sleep(1)
            leftSpin()
            time.sleep(0.1)
            turn = True

def sonic_mission3():
    speed = 0.5
    while True:
        dist = distance()
        if dist>100:
            speed = 1
            forward(speed)
        elif dist>20:
            speed = 0.5
            forward(speed)
        else:
            stopDCMotor()
            break
        time.sleep(0.1)
    
project()
