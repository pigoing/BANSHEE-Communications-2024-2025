import cv2
import numpy as np
from simple_pid import PID
import RPi.GPIO as GPIO
import time

# Set up GPIO for servo control
GPIO.setmode(GPIO.BCM)
PAN_SERVO_PIN = 13  # Pin for SG90 servo (pan)
TILT_SERVO_PIN = 18  # Pin for SG51 servo (tilt)

GPIO.setup(PAN_SERVO_PIN, GPIO.OUT)
GPIO.setup(TILT_SERVO_PIN, GPIO.OUT)

# Set PWM frequency to 50Hz for both servos
pan_pwm = GPIO.PWM(PAN_SERVO_PIN, 50)
tilt_pwm = GPIO.PWM(TILT_SERVO_PIN, 50)

# Start PWM with neutral positions
pan_pwm.start(7.5)
tilt_pwm.start(7.5)

# Function to convert angles to duty cycles for SG90 (pan)
def angle_to_duty_cycle_sg90(angle):
    return 5 + (angle / 180.0) * 5

# Function to convert angles to duty cycles for SG51 (tilt)
def angle_to_duty_cycle_sg51(angle):
    # You can adjust this range if needed for the SG51 behavior
    return 5 + (angle / 180.0) * 5

# Set up PID controllers with smoother values
pid_pan = PID(Kp=0.3, Ki=0.001, Kd=0.15, setpoint=0)
pid_tilt = PID(Kp=0.3, Ki=0.001, Kd=0.15, setpoint=0)

# Limit PID output to between -60 and 60 degrees
pid_pan.output_limits = (-60, 60)
pid_tilt.output_limits = (-60, 60)

# Set a deadband to prevent jitter from small errors
DEADBAND = 20  # Increase deadband to reduce small movements

# Limit the speed of the servo movements
MAX_SERVO_STEP = 2  # Degrees per update (slow down the movement)

# ArUco marker detection setup
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters_create()

# Open camera
cap = cv2.VideoCapture(0)

# Initialize servo positions
pan_angle = 90
tilt_angle = 90

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Convert frame to grayscale for marker detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        if ids is not None:
            # Get the first detected marker
            marker_corners = corners[0][0]

            # Calculate the center of the marker
            x_marker = int((marker_corners[0][0] + marker_corners[2][0]) / 2)
            y_marker = int((marker_corners[0][1] + marker_corners[2][1]) / 2)

            # Get the frame center
            frame_height, frame_width = gray.shape
            x_center = frame_width // 2
            y_center = frame_height // 2

            # Calculate the errors
            error_x = x_marker - x_center
            error_y = y_marker - y_center

            # Check if errors are significant enough to adjust servos
            if abs(error_x) > DEADBAND:
                pan_correction = pid_pan(error_x)
            else:
                pan_correction = 0

            if abs(error_y) > DEADBAND:
                tilt_correction = pid_tilt(error_y)
            else:
                tilt_correction = 0

            # Calculate new angles but limit movement speed
            pan_angle = np.clip(pan_angle + np.sign(pan_correction) * min(MAX_SERVO_STEP, abs(pan_correction)), 0, 180)
            tilt_angle = np.clip(tilt_angle + np.sign(tilt_correction) * min(MAX_SERVO_STEP, abs(tilt_correction)), 0, 180)

            # Update servo positions with duty cycles for respective servos
            pan_pwm.ChangeDutyCycle(angle_to_duty_cycle_sg90(pan_angle))
            tilt_pwm.ChangeDutyCycle(angle_to_duty_cycle_sg51(tilt_angle))

            # Draw the marker and the center points
            cv2.aruco.drawDetectedMarkers(frame, corners)
            cv2.circle(frame, (x_marker, y_marker), 5, (0, 255, 0), -1)
            cv2.circle(frame, (x_center, y_center), 5, (255, 0, 0), -1)

        # Show the video feed with marker tracking
        cv2.imshow('ArUco Marker Tracking', frame)

        # Break the loop on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Clean up resources
    pan_pwm.stop()
    tilt_pwm.stop()
    GPIO.cleanup()
    cap.release()
    cv2.destroyAllWindows()
