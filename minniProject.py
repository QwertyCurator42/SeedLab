from smbus2 import SMBus
from time import sleep
from random import random
from cv2 import aruco
import numpy as np
import time
import cv2
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import threading
import queue


#definitions for camera size
HEIGHT = 480
WIDTH = 640
fov = 34.25

##################################################
# LCD RPi hat must be installed for code to work #
##################################################
#definitions for supplied lcd
lcd_columns = 16
lcd_rows = 2
LCD = board.I2C()

# Offset value required for I2C communication
offset = 1

################################################################
# Arduino must be wired to SCL, SDA and GND as shown in README #
################################################################
#definitions for communicating with arduino
ARD = SMBus(1)
ARD_ADDR = 8

#defining q in order to use the Queue for threading to enable
#the LCD to function in parallel with driving motors
q = queue.Queue()

#define lcd in order to easily use methods such as color,message, and clear
lcd = character_lcd.Character_LCD_RGB_I2C(LCD, lcd_columns, lcd_rows)
#reset LCD prior to operations
lcd.clear()

#global dispText
dispText = "No marker found"

def displayLCD():
    curDisp = ""
    global dispText
    while True:
        #print("curStr: %s\ndispStr: %s" %(curDisp, dispText))
        # if the desired display text changes, clear display and update message
        if dispText != curDisp:
            lcd.clear()
            lcd.color = [100,0,0]
            lcd.message = "%s" %(dispText)
            curDisp = dispText
            # update message depending on which movements are desired
            if dispText != "No marker found":
                # transmit desired movement pattern to Arduino over I2C
                codeWord = int(dispText[16] + dispText[18])
                if codeWord == 11:
                    ARD.write_byte_data(ARD_ADDR, register=offset, value=3)
                elif codeWord == 10:
                    ARD.write_byte_data(ARD_ADDR, register=offset, value=4)
                elif codeWord == 1:
                    ARD.write_byte_data(ARD_ADDR, register=offset, value=2)
                elif codeWord == 00:
                    ARD.write_byte_data(ARD_ADDR, register=offset, value=1)
                print(codeWord)
        # make thread sleep for transmission time as well as stabalization of system
        time.sleep(0.1)

def continueDetect():
    global dispText

    # setup OpenCV camera
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    camera = cv2.VideoCapture(0)  # Initialize the camera
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

    # Make parameters for ArUco detect
    parameters = aruco.DetectorParameters_create()
    parameters.minMarkerPerimeterRate = 0.03

    # Let camera warmup
    sleep(0.1)
    try:
        while True:
            ret, frame = camera.read()  # Take an image
            grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Make the image greyscale for ArUco detection

            # Detect user exit key
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break

            # Detect ArUco markers
            corners, ids, rejected = aruco.detectMarkers(grey, aruco_dict, parameters=parameters)
            overlay = cv2.cvtColor(grey, cv2.COLOR_GRAY2RGB)
            overlay = aruco.drawDetectedMarkers(overlay, corners, borderColor=4)
            if ids is not None:
                ids = ids.flatten()
                for (outline, id) in zip(corners, ids):
                    markerCorners = outline.reshape((4, 2))
                    center = (
                        int(np.mean([point[0] for point in markerCorners])),
                        int(np.mean([point[1] for point in markerCorners]))
                    )
                    findQuad(center)
                    overlay = cv2.putText(overlay, str(id), (int(markerCorners[0, 0]), int(markerCorners[0, 1]) - 15),
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            if ids is None:
                dispText = "No marker found"

            # Display overlay to user
            cv2.imshow("overlay", overlay)

    except KeyboardInterrupt:
        # In case of error, release cleanly
        camera.release()

def findQuad(center):
    global dispText
    difference = np.array(center) - np.array([WIDTH / 2, HEIGHT / 2])  # Subtract the middle of the screen size
    if difference[0] > 0 and difference[1] > 0:
        dispText = "Goal Position:\n 0 1"
    elif difference[0] > 0 and difference[1] < 0:
        dispText = "Goal Position:\n 0 0"
    elif difference[0] < 0 and difference[1] > 0:
        dispText = "Goal Position:\n 1 1 "
    elif difference[0] < 0 and difference[1] < 0:
        dispText = "Goal Position:\n 1 0"
    else:
        dispText = "No marker found"
    angle = computeAngle(center)
    dispText += f" Angle: {angle:.2f}°"
    print(dispText)

def computeAngle(center):
    dx = center[0] - WIDTH/2
    
    # Normalize the deviation
    normalized_deviation = dx / (WIDTH/2)
    
    # Convert normalized deviation to angle
    angle = fov * normalized_deviation

    return -angle  # Negate the angle so that left is positive and right is negative

if __name__ == '__main__':
    continueDetect()
            