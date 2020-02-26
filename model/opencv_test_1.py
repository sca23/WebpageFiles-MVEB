from multiprocessing import Manager
from multiprocessing import Process
from imutils.video import VideoStream
##from reference.pid import PID
from simple_pid import PID
import imutils
import argparse
import signal
import time
import sys
import pantilthat as pth
##import os
import cv2
import RPi.GPIO as GPIO

prev = 0
# define the range for the motors
servoRange = (45, 215)

#define Servos GPIOs
panServo = 27
tiltServo = 17

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(panServo, GPIO.OUT)
GPIO.setup(tiltServo, GPIO.OUT)
# Initialize angle servos at 90-90 position
global panAngle
panAngle = 90
global tiltAngle
tiltAngle = 90

def obj_center(objX, objY, centerX, centerY):
    print("Waiting for for camera to warmup...")
    vs = VideoStream(0).start()
    time.sleep(1.0)
    
    colorLower =(20,100,100)
    colorUpper = (40,255,255)

    while True:
        # grab the next frame from the video stream, Invert 180o, resize the
        # frame, and convert it to the HSV color space
        frame = vs.read()
        frame = imutils.resize(frame, width=360, height=360)
        frame = imutils.rotate(frame, angle=180)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # calculate the center of the frame as this is where we will
        # try to keep the object
        (H,W) = frame.shape[:2]
        centerX.value = W//2
        centerY.value = H//2
        cv2.circle(frame, (centerX.value, centerY.value), 5, (0, 255, 0), -1)
        # construct a mask for the object color, then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, colorLower, colorUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        # find contours in the mask and initialize the current
        # (x, y) center of the object
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        center = None
        # only proceed if at least one contour was found
        if len(cnts) > 0:
                #find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                (objX.value, objY.value) = center
                # only proceed if the radius meets a minimum size
                if radius > 10:
                        #Draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                        cv2.circle(frame, center, 5, (0, 0, 255), -1)

##        print("[INFO] center->{0}\n".format(centerY.value))
##        print("[INFO] object->{0}\n".format(objY.value))

        # show the frame to our screen
        cv2.imshow("Frame", frame)
        # if [ctrl+c] key is pressed, stop the loop
        key = cv2.waitKey(1) & 0xFF
        if key == 67:
                positionServo(panServo, 90)
                positionServo(tiltServo, 90)
                GPIO.cleanup()
                cv2.destroyAllWindows()
                vs.stop()
                break


def pid_process(output, p, i, d, objCoord, centerCoord):
##    # signal trap to handle keyboard interrupt
##    signal.signal(signal.SIGINT, signal_handler)
    
    # create a PID and initialize it
    pid = PID(p.value, i.value, d.value, setpoint = centerCoord.value)
    pid.sample_time = 2.0
    pid.auto_mode = True
##    p.initialize()
    
    # loop indefinitely
    while True:
        output.value = pid(objCoord.value)
##            # calculate the error
##            error = centerCoord.value - objCoord.value
##
##            # update the value
##            output.value = p.update(error)
##            #print("Output: {0}\n".format(output.value))
##            #print("centerY {0} objectY {1}\n".format(centerCoord.value,objCoord.value))
##
##            #time.sleep(0.2)

#position servo
def positionServo(servo,angle):
    global prev
##    os.system("python angleServoCtrl.py " + str(servo) + " " + str(angle))
    assert angle >=45 and angle <= 215
    pwm = GPIO.PWM(servo, 50)
    pwm.start(prev)
    dutyCycle = angle / 18. + 3.
    pwm.ChangeDutyCycle(dutyCycle)
    time.sleep(0.3)
    pwm.stop()
    prev = dutyCycle
    time.sleep(1.7)


def mapServo(s,y):
    global tiltAngle
    while True:

        # the pan and tilt angles are reversed
######        panAngle = -1 * pan.value
        tiltAngle =  -1 * round(y.value)
######        print("op:{0}\n".format(tiltAngle))
######        

        # if the pan angle is within the range, pan
######        if in_range(panAngle, servoRange[0], servoRange[1]):
######            print("pan->{0}\n".format(90+panAngle))
######            positionServo(panServo, 90+panAngle)

##                #pth.pan(panAngle)

##        # if the tilt angle is within the range, tilt
##        print("PID: {0}\n".format(90+tiltAngle))
        if in_range(tiltAngle, servoRange[0], servoRange[1]):
            print("tilt->{0}\n".format(tiltAngle))
            positionServo(tiltServo, tiltAngle)
##            time.sleep(0.7)
            #pth.tilt(tiltAngle)

        
 
def in_range(val, start, end):
	# determine the input vale is in the supplied range
	return (val >= start and val <= end)

# check to see if this is the main body of execution
if __name__ == "__main__":
        # construct the argument parser and parse the arguments
        ap = argparse.ArgumentParser()
        args = vars(ap.parse_args())
        # positioning Pan/Tilt servos at initial position
        positionServo (panServo, 90)
        positionServo (tiltServo, 90)   

        # start a manager for managing process-safe variables
        with Manager() as manager:
                # set integer values for the object center (x, y)-coordinates
                centerX = manager.Value("i", 0)
                centerY = manager.Value("i", 0)
                # set integer values for the object's (x, y)-coordinates
                objX = manager.Value("i", 0)
                objY = manager.Value("i", 0)
                #set PID values for tilting
                tiltP = manager.Value("f", 1)#0.271)
                tiltI = manager.Value("f", 0)#0.0023)
                tiltD = manager.Value("f",0)#0.064)
                #pth.servo_enable(1, True)
                #pth.servo_enable(1, True)
                
                #tilt = manager.Value("i", 90)
                y = manager.Value("i", 0)
                s = manager.Value("i", 0)
                processObjectCenter = Process(target=obj_center,
                                              args=(objX, objY, centerX, centerY))
                processTilting = Process(target=pid_process,
                                         args=(y,tiltP,tiltI,tiltD,objY,centerY))
                processMapServo = Process(target=mapServo,
                                          args=(s,y))
                
                processObjectCenter.start()
                processTilting.start()
                processMapServo.start()
                
                processObjectCenter.join()
                processTilting.join()
                processMapServo.join()
