from dronekit import connect, VehicleMode, LocationGlobalRelative
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import math

# Connect to the vehicle
print("Connecting to vehicle...")
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Initialize video capture
vid = cv2.VideoCapture(0)
vid.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
print("Video started")

# ArUco marker dictionary and parameters
arucoDict = aruco.Dictionary_get(aruco.DICT_7X7_50)
arucoParam = aruco.DetectorParameters_create()

# Function to calculate distance between two GPS points
def get_distance_meters(a_location, b_location):
    dlat = b_location.lat - a_location.lat
    dlong = b_location.lon - a_location.lon
    return math.sqrt((dlat * 1.113195e5) ** 2 + (dlong * 1.113195e5) ** 2)

# Function to move the drone to a specific GPS location
def goto_location(target_location, groundspeed=1):
    vehicle.simple_goto(target_location, groundspeed=groundspeed)
    while vehicle.mode.name == "GUIDED":
        current_distance = get_distance_meters(vehicle.location.global_relative_frame, target_location)
        print(f"Distance to target: {current_distance:.2f} meters")
        if current_distance < 1:  # Within 1 meter of target
            print("Reached target location")
            break
        time.sleep(1)

# Function to detect ArUco marker
def find_aruco_marker(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    bbox, ids, _ = aruco.detectMarkers(gray, arucoDict, parameters=arucoParam)
    if ids is not None:
        aruco.drawDetectedMarkers(frame, bbox)
        return bbox, ids
    return None, None

# Function to calculate marker offset in frame
def calculate_offset(bbox, frame_shape):
    top_left = bbox[0][0][0][0], bbox[0][0][0][1]
    bottom_right = bbox[0][0][2][0], bbox[0][0][2][1]
    marker_center = ((top_left[0] + bottom_right[0]) / 2, (top_left[1] + bottom_right[1]) / 2)

    frame_center = (frame_shape[1] / 2, frame_shape[0] / 2)
    offset_x = marker_center[0] - frame_center[0]
    offset_y = marker_center[1] - frame_center[1]

    return offset_x, offset_y

# Function to convert frame offset to GPS adjustments
def offset_to_gps(offset_x, offset_y, altitude):
    scaling_factor = 1e-6 / altitude
    delta_lat = -offset_y * scaling_factor
    delta_lon = offset_x * scaling_factor
    return delta_lat, delta_lon

# Arming and taking off
print("Arming the drone...")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed:
    print("Waiting for arming...")
    time.sleep(1)

print("Taking off...")
target_altitude = 10  # Fly to 10 meters
vehicle.simple_takeoff(target_altitude)

while True:
    print(f"Altitude: {vehicle.location.global_relative_frame.alt:.2f}")
    if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
        print("Target altitude reached")
        break
    time.sleep(1)

# Search for the marker and align
print("Searching for ArUco marker...")
while True:
    ret, frame = vid.read()
    if not ret:
        break

    bbox, ids = find_aruco_marker(frame)
    if ids is not None:
        print("Marker detected. Aligning drone...")

        # Calculate marker offset
        offset_x, offset_y = calculate_offset(bbox, frame.shape)
        print(f"Offset X: {offset_x}, Offset Y: {offset_y}")

        # Convert offset to GPS adjustments
        delta_lat, delta_lon = offset_to_gps(offset_x, offset_y, vehicle.location.global_relative_frame.alt)

        # Adjust drone's position
        current_location = vehicle.location.global_relative_frame
        target_location = LocationGlobalRelative(
            current_location.lat + delta_lat,
            current_location.lon + delta_lon,
            target_altitude
        )
        goto_location(target_location)

        # Check if aligned
        if abs(offset_x) < 20 and abs(offset_y) < 20:  # Alignment threshold
            print("Drone aligned with marker.")
            break

    # Display camera feed
    cv2.imshow("ArUco Marker Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Land the drone
print("Landing...")
vehicle.mode = VehicleMode("LAND")

while vehicle.armed:
    print("Landing in progress...")
    time.sleep(1)

# Release resources
vid.release()
cv2.destroyAllWindows()
vehicle.close()
print("Mission complete.")
