from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
import cv2
import cv2.aruco as aruco
import numpy as np

# Connect to the drone
print("Connecting to vehicle...")
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Camera and ArUco marker initialization
arucoDict = aruco.Dictionary_get(aruco.DICT_7X7_50)
arucoParam = aruco.DetectorParameters_create()
vid = cv2.VideoCapture(0)

# Function to calculate the distance between two points (latitude and longitude)
def get_distance_meters(a_location, b_location):
    dlat = b_location.lat - a_location.lat
    dlong = b_location.lon - a_location.lon
    return math.sqrt((dlat * 1.113195e5) ** 2 + (dlong * 1.113195e5) ** 2)

# Function to move the drone to a specific location
def goto_location(target_location, groundspeed=1):
    vehicle.simple_goto(target_location, groundspeed=groundspeed)
    while vehicle.mode.name == "GUIDED":
        current_distance = get_distance_meters(vehicle.location.global_relative_frame, target_location)
        if current_distance < 1:  # Within 1 meter of target
            print("Reached target location")
            break
        time.sleep(1)

# ArUco marker detection function
def find_aruco_marker(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    bbox, ids, _ = aruco.detectMarkers(gray, arucoDict, parameters=arucoParam)
    if ids is not None:
        aruco.drawDetectedMarkers(frame, bbox)
        return bbox, ids
    return None, None

# Main code
print("Arming the drone...")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

# Wait for the drone to arm
while not vehicle.armed:
    print("Waiting for arming...")
    time.sleep(1)

print("Taking off...")
target_altitude = 10  # Fly to 10 meters
vehicle.simple_takeoff(target_altitude)

# Wait until the drone reaches the target altitude
while True:
    print("Altitude:", vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
        print("Target altitude reached")
        break
    time.sleep(1)

print("Searching for ArUco marker...")
while True:
    ret, frame = vid.read()
    if not ret:
        break

    bbox, ids = find_aruco_marker(frame)
    if ids is not None:
        print("Marker detected. Aligning drone...")
        # Calculate marker center
        marker_center_x = (bbox[0][0][0][0] + bbox[0][0][2][0]) / 2
        marker_center_y = (bbox[0][0][0][1] + bbox[0][0][2][1]) / 2
        frame_center_x = frame.shape[1] / 2
        frame_center_y = frame.shape[0] / 2

        # Calculate offsets
        offset_x = marker_center_x - frame_center_x
        offset_y = marker_center_y - frame_center_y

        # Adjust drone position based on offsets
        if abs(offset_x) > 20 or abs(offset_y) > 20:  # Threshold for alignment
            print(f"Adjusting position: offset_x={offset_x}, offset_y={offset_y}")
            # Calculate target position
            current_location = vehicle.location.global_relative_frame
            target_location = LocationGlobalRelative(
                current_location.lat - (offset_y * 1e-6),
                current_location.lon + (offset_x * 1e-6),
                target_altitude
            )
            goto_location(target_location)
        else:
            print("Drone aligned with marker.")
            break

print("Landing...")
vehicle.mode = VehicleMode("LAND")

# Wait for landing
while vehicle.armed:
    print("Landing in progress...")
    time.sleep(1)

print("Landed. Disarming...")
vehicle.armed = False

# Release resources
vid.release()
cv2.destroyAllWindows()
vehicle.close()
print("Mission complete.")