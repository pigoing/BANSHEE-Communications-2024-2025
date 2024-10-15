import cv2
import aruco_draw
import math
import pygame
from pygame.locals import *

import RPi.GPIO as GPIO
import time

# Define GPIO pins for gimbal movement
servo_pin_x = 18  # GPIO pin 18 (physical pin 12) for horizontal movement
servo_pin_y = 13  # GPIO pin 13 (physical pin 33) for vertical movement



# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(x_movement, GPIO.OUT)
GPIO.setup(y_movement, GPIO.OUT)

# Set up PWM control
pan_pwm = GPIO.PWM(servo_pin_x, 50)  # 50Hz pulse
tilt_pwm = GPIO.PWM(servo_pin_y, 50)  # 50Hz pulse

# Start PWM with neutral position
pan_pwm.start(7.5)  # 7.5% duty cycle -> middle position (90 degrees)
tilt_pwm.start(7.5)  # 7.5% duty cycle -> middle position (90 degrees)

# Try testing with a simple script to find the correct range:
for duty in range(25, 125):  # Range 2.5% to 12.5%
    pan_pwm.ChangeDutyCycle(duty / 10.0)
    time.sleep(0.5)  # Give the servo time to move


# Function to move the gimbal servos
def move_gimbal(x_movement, y_movement):
    # Scale the movements to a valid duty cycle range
    x_duty = 7.5 + (x_movement / 1000.0)  # Adjust scaling as needed
    y_duty = 7.5 + (y_movement / 1000.0)  # Adjust scaling as needed

    # Clamp duty cycles to avoid out-of-range values that could seize the servo
    x_duty = max(2.5, min(12.5, x_duty))
    y_duty = max(2.5, min(12.5, y_duty))

    # Move the servos
    pan_pwm.ChangeDutyCycle(x_duty)
    tilt_pwm.ChangeDutyCycle(y_duty)

    time.sleep(0.05)  # Allow time for the servo to move


# Cleanup after program is done
def cleanup():
    pan_pwm.stop()
    tilt_pwm.stop()
    GPIO.cleanup()
    

cv2.namedWindow("Test Window")
vc = cv2.VideoCapture(0)

# Parameters
x_P = 0.3
y_P = 0.3
box_size = 200
land = False

# Scaling factor for resizing the window
scale_factor = 2  # Adjust this value to make the window larger or smaller

while not land:
    # Get frame
    rval, img = vc.read()
    if not rval:
        print("Failed to capture frame")
        break

    # Resize the image
    resized_img = cv2.resize(img, (0, 0), fx=scale_factor, fy=scale_factor)

    # ArUco marker detection
    aruco_dict = cv2.aruco.dictionary_get(cv2.aruco.DICT_7X7_250)
    aruco_params = cv2.aruco.DetectorParameters_create()
    corners, ids, _ = cv2.aruco.detectMarkers(resized_img, aruco_dict, parameters=aruco_params)

    # Draw markers if detected
    aruco_draw.centerloc(resized_img, corners, ids)

    # If markers are detected
    if ids is not None:
        a_id = ids[0][0]
        text = f"AR Marker Dict 7x7, ID {a_id} - Processing"

        # Calculate average position and area for ID 0
        if a_id == 0:
            corner = corners[0][0]
            avg_x, avg_y = corner[:, 0].mean(), corner[:, 1].mean()

            bottom_left, top_left, top_right, bottom_right = corner

            diag1 = math.dist(bottom_left, top_left)
            diag2 = math.dist(top_left, bottom_right)
            area = diag1 * diag2 / 2

            # Drawing centerlines
            h, w = resized_img.shape[:2]
            center_x, center_y = w // 2, h // 2
            avg_x, avg_y = int(avg_x), int(avg_y)
            x_dist = avg_x - center_x
            y_dist = avg_y - center_y

            # Visual feedback
            cv2.line(resized_img, (0, center_y), (w, center_y), (0, 255, 0), 10)
            cv2.line(resized_img, (0, avg_y), (w, avg_y), (0, 0, 255), 10)
            cv2.line(resized_img, (center_x, 0), (center_x, h), (0, 255, 0), 10)
            cv2.line(resized_img, (avg_x, 0), (avg_x, h), (0, 0, 255), 10)

            # Bounding box
            half_box_size = box_size // 2
            cv2.rectangle(resized_img, (center_x - half_box_size, center_y - half_box_size),
                          (center_x + half_box_size, center_y + half_box_size), (255, 0, 0), 10)

            # Apply proportional movement scaling
            x_movement = int(x_dist * x_P)
            y_movement = int(y_dist * y_P)
            

            # Move gimbal to track the marker
            move_gimbal(x_movement, y_movement)
    else:
        text = "No marker detected."

    # Show text and image
    cv2.putText(resized_img, text, (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0))
    cv2.imshow("ArUco Marker Detection", resized_img)

    # Exit condition
    if cv2.waitKey(10) & 0xFF == ord('q'):
        land = True

cv2.destroyAllWindows()
