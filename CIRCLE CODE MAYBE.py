from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math

# Connect to the vehicle
print("Connecting to vehicle...")
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Function to calculate GPS offset based on meters
def get_gps_offset(original_location, dNorth, dEast):
    earth_radius = 6378137.0  # Earth radius in meters
    new_lat = original_location.lat + (dNorth / earth_radius) * (180 / math.pi)
    new_lon = original_location.lon + (dEast / (earth_radius * math.cos(math.radians(original_location.lat))) * (180 / math.pi))
    return LocationGlobalRelative(new_lat, new_lon, original_location.alt)

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

# Function to calculate distance between two GPS points
def get_distance_meters(a_location, b_location):
    dlat = b_location.lat - a_location.lat
    dlong = b_location.lon - a_location.lon
    return math.sqrt((dlat * 1.113195e5) ** 2 + (dlong * 1.113195e5) ** 2)

# Pre-arm sequence
print("Waiting for vehicle to initialize...")
while not vehicle.is_armable:
    print("Waiting for vehicle to become armable...")
    time.sleep(1)

# Arming the drone
print("Arming the drone...")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed:
    print("Waiting for arming...")
    time.sleep(1)

# Takeoff sequence
target_altitude = 10  # Fly to 10 meters
print(f"Taking off to {target_altitude} meters...")
vehicle.simple_takeoff(target_altitude)

timeout = time.time() + 30  # 30-second timeout
while True:
    print(f"Altitude: {vehicle.location.global_relative_frame.alt:.2f}")
    if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
        print("Target altitude reached")
        break
    if time.time() > timeout:
        print("Takeoff timeout reached! Check sensors or mode.")
        break
    time.sleep(1)

# Autonomous Circular Flight Path (~10 feet radius)
print("Starting circular flight path...")
waypoints = 8  # Number of points in the circle
radius_meters = 3  # ~10 feet
current_location = vehicle.location.global_relative_frame

for i in range(waypoints):
    angle = i * (360 / waypoints)
    dNorth = radius_meters * math.cos(math.radians(angle))
    dEast = radius_meters * math.sin(math.radians(angle))

    waypoint = get_gps_offset(current_location, dNorth, dEast)
    print(f"Flying to waypoint {i + 1}/{waypoints}: Lat {waypoint.lat}, Lon {waypoint.lon}")
    goto_location(waypoint)
    time.sleep(2)  # Small delay to stabilize at each point

# Return to center
print("Returning to original position...")
goto_location(current_location)
time.sleep(2)

# Landing sequence
print("Landing...")
vehicle.mode = VehicleMode("LAND")

while vehicle.armed:
    print("Landing in progress...")
    time.sleep(1)

# Close the connection
vehicle.close()
print("Mission complete.")
