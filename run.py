import cv2
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
reverseLeft = PWMOutputDevice(PWM_REVERSE_LEFT_PIN, True, 0, 1000)
forwardRight = PWMOutputDevice(PWM_FORWARD_RIGHT_PIN, True, 0, 1000)
reverseRight = PWMOutputDevice(PWM_REVERSE_RIGHT_PIN, True, 0, 1000)

# Initializing the ultrasonic sensor
gpio.setmode(gpio.BCM)
trigger = 20
echo = 21
gpio.setup(trigger, gpio.OUT)
gpio.setup(echo, gpio.IN)

def stop():
	forwardLeft.value = 0
	reverseLeft.value = 0
	forwardRight.value = 0
	reverseRight.value = 0
 
def forward(speed):
	forwardLeft.value = speed
	reverseLeft.value = 0
	forwardRight.value = speed
	reverseRight.value = 0
 
def reverse(speed):
	forwardLeft.value = 0
	reverseLeft.value = speed
	forwardRight.value = 0
	reverseRight.value = speed
    
def left(speed):
	forwardLeft.value = 1 - speed
	reverseLeft.value = 0
	forwardRight.value = 0.5
	reverseRight.value = 0
 
def right(speed):
	forwardLeft.value = 0.5
	reverseLeft.value = 0
	forwardRight.value = 1 - speed
	reverseRight.value = 0
    
def leftBack(speed):
	forwardLeft.value = 0
	reverseLeft.value = 1 - speed
	forwardRight.value = 0
	reverseRight.value = 1
 
def rightBack(speed):
	forwardLeft.value = 0
	reverseLeft.value = 1
	forwardRight.value = 0
	reverseRight.value = 1 - speed
 
def rightSpin():
    forwardLeft.value = 1.0
    reverseLeft.value = 0
    forwardRight.value = 0
    reverseRight.value = 1.0
    
def leftSpin():
    forwardLeft.value = 0
    reverseLeft.value = 1.0
    forwardRight.value = 1.0
    reverseRight.value = 0
    
def edge_detector(image):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    canny = cv2.Canny(blur, 50, 150)
    #_, canny = cv2.threshold(blur,100,255,cv2.THRESH_BINARY)
    return canny

def section(image): 
    height = image.shape[0] 
    polygons = np.array([ 
        [(0, height), (320, height), (320, 240), (0, 240)] 
        ]) 
    polygons = np.array([ 
        [(320, height), (640, height), (640, 240), (320, 240)] 
        ])
    polygons = np.array([ 
        [(0, height), (640, height), (640, 240), (0, 240)] 
        ])
    mask = np.zeros_like(image)  
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(image, mask)  
    return masked_image
    
def distance():
    gpio.output(trigger, 1)
    time.sleep(0.00001)
    gpio.output(trigger, 0)
    startTime = time.time()
    stopTime = time.time()
    while gpio.input(echo) == 0:
        startTime = time.time()
    while gpio.input(echo) == 1:
        stopTime = time.time()
    timeSpent = stopTime - startTime
    distance = (timeSpent * 34300) / 2
    return distance
        
def display_lines(image, lines): 
    line_image = np.zeros_like(image) 
    if lines is not None: 
        for line in lines: 
            line = line[0]
            x1, y1, x2, y2 = line
            cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
    return line_image

def get_slope(image, lines):
    if lines is not None:
        slopes = 0
        count = lines.size / 4
        for line in lines:
            line = line[0]
            x1, y1, x2, y2 = line
            y2 = 640 - y2
            y1 = 640 - y1
            if x2 > x1:
                slope = (y2 - y1)/(x2 - x1)
            elif x2 == x1:
                slope = 2
            else:
                slope = (y1 - y2)/(x1 - x2)
                
            if(abs(slope) > 50):
                count-=1
            else:
                slopes += slope
        return slopes/count
    else:
        return 1
    
def autoright(slope):
    if slope > 1:
        left(0.8)
    elif slope < 0.1:
        stop()
    else:
        right(1-slope*0.5)
    

def project():
    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))

    # allow the camera to warmup
    time.sleep(0.1)
    
    # capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        img = frame.array
        # process the frame
        img = edge_detector(img)
        img = section(img)
        lines = cv2.HoughLinesP(img, rho = 1, theta = np.pi/180, 
                                threshold = 20,  maxLineGap = 5)
        img = display_lines(img, lines)
        # show the frame
        cv2.imshow("Frame", img)
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        print(get_slope(img, lines))
        #autoturn(get_slope(img, lines))
        data = cv2.waitKey(1) & 0xFF
        # if the `q` key was pressed, break 
        if(data==ord(" ")):
            stop()
        elif(data==ord("w")):
            forward(speed)
        elif(data==ord("a")):
            left(1)
        elif(data==ord("d")):
            right(1)
        elif(data==ord("s")):
            reverse(speed)
        elif(data==ord("q")):
            stop()
            break
        
            
def motor_mission1():
    rightAngle = 0.3
    #forward for 2 seconds
    forward(1)
    time.sleep(2)
    #wait for 1 seconds
    stop()
    time.sleep(1)
    #turn right
    rightSpin()
    time.sleep(rightAngle)
    #go backward for 2 seconds
    reverse(1)
    time.sleep(2)
    #wait for 1 seconds
    stop()
    time.sleep(1)
    #turn left
    leftSpin()
    time.sleep(rightAngle)
    #stop the car
    stop()

def motor_mission2():
    fullAngle = 1.2
    #right rotate 360 degrees
    rightSpin()
    time.sleep(fullAngle)
    #wait for 1 seconds
    stop()
    time.sleep(1)
    #left rotate 360 degrees
    leftSpin()
    time.sleep(fullAngle)
    
def motor_mission3():
    #slowly go forward
    forward(0.4)
    time.sleep(1)
    #slightly increase the speed
    forward(0.6)
    time.sleep(1)
    #get the maximum speed
    forward(1)
    time.sleep(1)
    stop()
    
def motor_mission4():
    rightAngle = 1.25
    #turn right forward
    right(0.6)
    time.sleep(rightAngle)
    #forward for 1 seconds
    forward(1)
    time.sleep(1)
    #turn left forward
    rightAngle = 1.6
    left(0.6)
    time.sleep(rightAngle)
    #forward for 1 seconds
    forward(1)
    time.sleep(1)
    stop()
    
def motor_mission5():
    rightAngle = 0.9
    #turn right back
    rightBack(0.6)
    time.sleep(rightAngle)
    #backward for 1 seconds
    reverse(1)
    time.sleep(1)
    #turn left back
    rightAngle = 1.4
    leftBack(0.6)
    time.sleep(rightAngle)
    #backward for 1 seconds
    reverse(1)
    time.sleep(1)
    stop()
    
def sonic_mission1():
    while True:
        dist = distance()
        if(dist>30):
            forward(speed)
        elif(dist<30):
            reverse(speed)
        else:
            stop()        
        
def sonic_mission2():
    turn = False
    while True:
        dist = distance()
        if(dist>30):
            forward(speed)
            if turn == True:
                time.sleep(2)
                stop()
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
            stop()
            break
        time.sleep(0.1)
    
project()
