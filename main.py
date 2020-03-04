from picamer.array import PiRGBArray
from gpiozero import PWMOutputDevice
import time
# Motor A 
FORWARD_LEFT_PIN = 18	# IN1
REVERSE_LEFT_PIN = 23	# IN2
# Motor B
FORWARD_RIGHT_PIN = 24	# IN3 
REVERSE_RIGHT_PIN = 25  # IN4
# ultrasonic PIN
SONIC_TRIGGER = 20 	# 28
SONIC_ECHO = 21		# 29

# IR Line Tracor sensor
LEFT_TRACER_PIN = 10
RIGHT_TRACER_PIN = 11

# initialize motor pins 
forwardLeft = PWMOutputDevice(FORWARD_LEFT_PIN,True,0,1000)
reverseLeft = PWMOutputDevice(REVERSE_LEFT_PIN,True,0,1000)
forwardRight = PWMOutputDevice(FORWARD_RIGHT_PIN,True,0,1000)
reverseRight = PWMOutputDevice(REVERSE_RIGHT_PIN,True,0,1000)

# initalization ultrasonic sensor
gpio.setmode(gpio.BCM)
gpio.setup(SONIC_TRIGGER,gpio.OUT)
gpio.setup(SONIC_ECHO, gpio.IN)

# initalization IR sensor
gpio.setmode(gpio.BCM)
gpio.setup(LEFT_TRACER_PIN,gpio.IN)
gpio.setup(RIGHT_TRACER_PIN,gpio.IN)

# stop DC Motor
def stop():
	forwardRight.value = 0;
	forwardLeft.value = 0;
	reverseRight.value = 0;
	reverseLeft.value = 0;
# initalization DC Motor
def initDCMotor():
	forwardLeft.value = 1;
	forwardRight.value = 1;
	reverseLeft.Value = 1;
	reverseRight.value = 1
# go forward
def goForward(speed):
	forwardLeft.value = speed;
	forwardRight.value = speed;
	reverseLeft.value = 0;
	reverseRight.value = 0;
# go back
def goBackward(speed):
	forwardLeft.value = 0;
	reverseLeft.value = 1;
	forwardRight.value = 0;
	reverseRight.value = 1;

def goRight(speed):
	forwardLeft.value = speed;
	reverseLeft.value = 0;
	forwardRight.value = 0
	reverseRight.value = speed;

def goLeft(speed):
	forwardLeft.value = 0;
	reverseLeft.value = speed
	forwardRight.value = speed;
	reverseRight.value = 0
# smooth turn right or left
def smoothTurn(lspeed,rspeed):
	forwardLeft.value = lspeed
	reverseLeft.value = 0
	forwardRight.value = rspeed
	reverseRight.value = 0

def turnBack(lspeed,rspeed):
	forwardLeft.value = 0
	reverseLeft.value = lspeed
	forwardRight.value = 0
	reverseRight.value = rspeed

def getDistance():
	gpio.output(SONIC_TRIGGER, 1)
    time.sleep(0.00001)
    gpio.output(SONIC_TRIGGER, 0)
    startTime = time.time()
    stopTime = time.time()
    while gpio.input(echo) == 0:
        startTime = time.time()
    while gpio.input(echo) == 1:
        stopTime = time.time()
    timeSpent = stopTime - startTime
    distance = (timeSpent * 34300) / 2
    return distance

def lineTracer(ntime):
	# initialization left and right IR sensor
	leftTracer = gpio.output(LEFT_TRACER_PIN)
	rightTracer = gpio.output(RIGHT_TRACER_PIN)

	if (leftTracer == 0 and rightTracer == 1):
		goLeft();
	elif (rightTracer == 0 and leftTracer == 1):
		goRight();
	elif rightTracer == 1 and leftTracer == 1:
		stop();
	time.sleep(ntime)










