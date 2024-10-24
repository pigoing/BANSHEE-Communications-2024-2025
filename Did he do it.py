\import cv2
import numpy as np
from simple_pid import PID
import RPi.GPIO as GPIO
import time

# Set up GPIO for servo control
GPIO.setmode(GPIO.BCM)
PAN_SERVO_PIN = 13  # Pin for pan servo
TILT_SERVO_PIN = 18  # Pin for tilt servo

GPIO.setup(PAN_SERVO_PIN, GPIO.OUT)
GPIO.setup(TILT_SERVO_PIN, GPIO.OUT)

# Set PWM frequency to 50Hz for the servos
pan_pwm = GPIO.PWM(PAN_SERVO_PIN, 50)
tilt_pwm = GPIO.PWM(TILT_SERVO_PIN, 50)

# Start PWM with neutral positions
pan_pwm.start(7.5)
tilt_pwm.start(7.5)

# Function to convert angles to duty cycles
def angle_to_duty_cycle(angle):
    return 5 + (angle / 180.0) * 5

# Set up PID controllers with smoother values
pid_pan = PID(Kp=0.2, Ki=0.001, Kd=0.1, setpoint=0)
pid_tilt = PID(Kp=0.2, Ki=0.001, Kd=0.1, setpoint=0)

# Limit PID output to between -60 and 60 degrees for smoother movement
pid_pan.output_limits = (-60, 60)
pid_tilt.output_limits = (-60, 60)

# Set a deadband to prevent jitter from small errors
DEADBAND = 30  # Increase deadband to reduce small movements

# Limit the speed of the servo movements
MAX_SERVO_STEP = 1  # Degrees per update (slow down the movement)

# ArUco marker detection setup
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters_create()

# Open camera
cap = cv2.VideoCapture(0)

# Initialize servo positions
pan_angle = 90
tilt_angle = 90

# Smoothing factor for exponential moving average (EMA)
alpha = 0.1
smoothed_error_x = 0
smoothed_error_y = 0

# Buffer to average marker position over multiple frames
marker_positions_x = []
marker_positions_y = []
FRAME_BUFFER = 5  # Number of frames to average over

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

            # Add marker position to the buffer
            marker_positions_x.append(x_marker)
            marker_positions_y.append(y_marker)

            if len(marker_positions_x) > FRAME_BUFFER:
                marker_positions_x.pop(0)
                marker_positions_y.pop(0)

            # Average the marker positions over the last few frames
            x_marker = int(np.mean(marker_positions_x))
            y_marker = int(np.mean(marker_positions_y))

            # Get the frame center
            frame_height, frame_width = gray.shape
            x_center = frame_width // 2
            y_center = frame_height // 2

            # Calculate the errors
            error_x = x_marker - x_center
            error_y = y_marker - y_center

            # Calculate smoothed errors using exponential moving average (EMA)
            smoothed_error_x = alpha * error_x + (1 - alpha) * smoothed_error_x
            smoothed_error_y = alpha * error_y + (1 - alpha) * smoothed_error_y

            # Check if errors are significant enough to adjust servos
            if abs(smoothed_error_x) > DEADBAND:
                pan_correction = pid_pan(smoothed_error_x)
            else:
                pan_correction = 0

            if abs(smoothed_error_y) > DEADBAND:
                tilt_correction = pid_tilt(smoothed_error_y)
            else:
                tilt_correction = 0

            # Calculate new angles but limit movement speed
            pan_angle = np.clip(pan_angle + np.sign(pan_correction) * min(MAX_SERVO_STEP, abs(pan_correction)), 0, 180)
            tilt_angle = np.clip(tilt_angle + np.sign(tilt_correction) * min(MAX_SERVO_STEP, abs(tilt_correction)), 0, 180)

            # Update servo positions with duty cycles
            pan_pwm.ChangeDutyCycle(angle_to_duty_cycle(pan_angle))
            tilt_pwm.ChangeDutyCycle(angle_to_duty_cycle(tilt_angle))

            # Draw the marker and the center points
            cv2.aruco.drawDetectedMarkers(frame, corners)
            cv2.circle(frame, (x_marker, y_marker), 5, (0, 255, 0), -1)
            cv2.circle(frame, (x_center, y_center), 5, (255, 0, 0), -1)

        # Show the video feed with marker tracking
        cv2.imshow('ArUco Marker Tracking', frame)

        # Break the loop on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Add a small delay to slow down the loop and reduce jitter
        time.sleep(0.05)

finally:
    # Clean up resources
    pan_pwm.stop()
    tilt_pwm.stop()
    GPIO.cleanup()
    cap.release()
    cv2.destroyAllWindows()
