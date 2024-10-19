import cv2
import aruco_draw
import math
import pygame
from pygame.locals import *

import RPi.GPIO as GPIO
import time

# Initialize GPIO for servos
servo_pin_y = 13  # Vertical (Y-axis) servo
servo_pin_x = 19  # Horizontal (X-axis) servo

GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin_y, GPIO.OUT)
GPIO.setup(servo_pin_x, GPIO.OUT)

# Setup PWM for both servos at 50Hz
pwm_y = GPIO.PWM(servo_pin_y, 50)
pwm_x = GPIO.PWM(servo_pin_x, 50)
pwm_y.start(0)
pwm_x.start(0)

# Function to control servo angle
def set_servo_angle(servo_pwm, angle):
    angle = max(0, min(180, angle))  # Constrain angle
    duty_cycle = 2.5 + (angle / 180) * 10
    servo_pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.05)  # Small delay for smooth operation

# Parameters
x_P = 0.3  # Proportional gain for X-axis movement
y_P = 0.3  # Proportional gain for Y-axis movement
box_size = 200  # Bounding box size
land = False  # Loop control flag

# Initial servo angles
angle_x, angle_y = 90, 90  # Start at neutral position (90 degrees)

# Scaling factor for resizing the window
scale_factor = 2

# Initialize camera
cv2.namedWindow("ArUco Marker Detection")
vc = cv2.VideoCapture(0)

try:
    while not land:
        # Get a frame from the camera
        rval, img = vc.read()
        if not rval:
            print("Failed to capture frame")
            break

        # Convert to grayscale and resize the frame
        greyscale_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        resized_img = cv2.resize(greyscale_img, (0, 0), fx=scale_factor, fy=scale_factor)

        # Detect ArUco markers
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_250)
        aruco_params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(resized_img, aruco_dict, parameters=aruco_params)

        # Draw detected markers
        aruco_draw.centerloc(resized_img, corners, ids)

        if ids is not None:  # Marker detected
            a_id = ids[0][0]
            text = f"AR Marker Dict 7x7, ID {a_id} - Processing"

            if a_id == 0:  # Process only marker with ID 0
                corner = corners[0][0]
                avg_x, avg_y = corner[:, 0].mean(), corner[:, 1].mean()

                # Calculate frame center and distances
                h, w = resized_img.shape[:2]
                center_x, center_y = w // 2, h // 2
                avg_x, avg_y = int(avg_x), int(avg_y)

                x_dist = avg_x - center_x
                y_dist = avg_y - center_y

                # Proportional control: Convert distances to movements
                x_movement = int(x_dist * x_P)
                y_movement = int(y_dist * y_P)

                # Update servo angles
                angle_x = max(0, min(180, angle_x - x_movement))
                angle_y = max(0, min(180, angle_y + y_movement))  # Inverted Y-axis

                # Move servos to new positions
                set_servo_angle(pwm_x, angle_x)
                set_servo_angle(pwm_y, angle_y)

                # Visual feedback: Draw lines and bounding box
                half_box_size = box_size // 2
                cv2.rectangle(resized_img, (center_x - half_box_size, center_y - half_box_size),
                              (center_x + half_box_size, center_y + half_box_size), (255, 0, 0), 10)
                cv2.line(resized_img, (0, center_y), (w, center_y), (0, 255, 0), 10)
                cv2.line(resized_img, (0, avg_y), (w, avg_y), (0, 0, 255), 10)
                cv2.line(resized_img, (center_x, 0), (center_x, h), (0, 255, 0), 10)
                cv2.line(resized_img, (avg_x, 0), (avg_x, h), (0, 0, 255), 10)

        else:  # No marker detected
            text = "No marker detected."
            # Small sweep to keep servos active
            for i in range(0, 180, 10):
                set_servo_angle(pwm_x, i)
                set_servo_angle(pwm_y, i)
                time.sleep(0.1)

        # Display the frame with annotations
        cv2.putText(resized_img, text, (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0))
        cv2.imshow("ArUco Marker Detection", resized_img)

        # Exit on 'q' key press
        if cv2.waitKey(10) & 0xFF == ord('q'):
            land = True

finally:
    # Cleanup
    vc.release()
    cv2.destroyAllWindows()
    pwm_y.stop()
    pwm_x.stop()
    GPIO.cleanup()
