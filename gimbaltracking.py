import cv2
import math
import RPi.GPIO as GPIO
import time

# GPIO setup for servos
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Servo pins (adjust to your wiring)
pan_pin =   # Horizontal servo (pan)
tilt_pin =   # Vertical servo (tilt)

GPIO.setup(pan_pin, GPIO.OUT)
GPIO.setup(tilt_pin, GPIO.OUT)

# Set up PWM for the servos
pan_servo = GPIO.PWM(pan_pin, 50)  # 50Hz pulse for pan
tilt_servo = GPIO.PWM(tilt_pin, 50)  # 50Hz pulse for tilt

# Start servos at neutral position (90 degrees)
pan_servo.start(7.5)  # 90 degrees
tilt_servo.start(7.5)  # 90 degrees

# Function to move the servos based on error
def move_servo(servo, error, proportional_gain=0.005):
    movement = 7.5 + error * proportional_gain
    movement = max(2.5, min(12.5, movement))  # Limit to servo range (2.5 to 12.5 for 0 to 180 degrees)
    servo.ChangeDutyCycle(movement)
    time.sleep(0.02)  # Small delay to give the servo time to move

# Camera setup
cv2.namedWindow("preview")
vc = cv2.VideoCapture(1)  # Change the index if using a different camera

# Parameters
x_P = 0.005  # Proportional gain for x-axis (horizontal)
y_P = 0.005  # Proportional gain for y-axis (vertical)
box_size = 200
land = False

# Scaling factor for resizing the window (adjust as necessary)
scale_factor = 2

while not land:
    # Capture frame
    rval, img = vc.read()
    if not rval:
        print("Failed to capture frame")
        break

    # Resize the image
    resized_img = cv2.resize(img, (0, 0), fx=scale_factor, fy=scale_factor)

    # ArUco marker detection
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_250)
    aruco_params = cv2.aruco.DetectorParameters_create()
    corners, ids, _ = cv2.aruco.detectMarkers(resized_img, aruco_dict, parameters=aruco_params)

    # If markers are detected
    if ids is not None:
        a_id = ids[0][0]

        # Calculate average position and area for ID 0
        if a_id == 0:
            corner = corners[0][0]
            avg_x, avg_y = corner[:, 0].mean(), corner[:, 1].mean()

            bottom_left, top_left, top_right, bottom_right = corner

            diag1 = math.dist(bottom_left, top_left)
            diag2 = math.dist(top_left, bottom_right)
            area = diag1 * diag2 / 2

            # Drawing centerlines for visual feedback
            h, w = resized_img.shape[:2]
            center_x, center_y = w // 2, h // 2
            avg_x, avg_y = int(avg_x), int(avg_y)

            # Calculate error from the center of the frame
            x_error = avg_x - center_x
            y_error = avg_y - center_y

            # Move servos based on the calculated error
            move_servo(pan_servo, x_error, x_P)  # Adjust horizontal servo (pan)
            move_servo(tilt_servo, y_error, y_P)  # Adjust vertical servo (tilt)

            # Visual feedback
            cv2.line(resized_img, (0, center_y), (w, center_y), (0, 255, 0), 10)  # Horizontal center line
            cv2.line(resized_img, (0, avg_y), (w, avg_y), (0, 0, 255), 10)  # Current y-position
            cv2.line(resized_img, (center_x, 0), (center_x, h), (0, 255, 0), 10)  # Vertical center line
            cv2.line(resized_img, (avg_x, 0), (avg_x, h), (0, 0, 255), 10)  # Current x-position

            # Bounding box for the target area
            half_box_size = box_size // 2
            cv2.rectangle(resized_img, (center_x - half_box_size, center_y - half_box_size),
                          (center_x + half_box_size, center_y + half_box_size), (255, 0, 0), 10)
    else:
        text = "No marker detected."
        cv2.putText(resized_img, text, (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0))

    # Display the resulting frame
    cv2.imshow("ArUco Marker Detection", resized_img)

    # Exit condition (press 'q' to quit)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        land = True

# Clean up
cv2.destroyAllWindows()
vc.release()
pan_servo.stop()
tilt_servo.stop()
GPIO.cleanup()
