from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import serial.tools.list_ports

# -------------------- Pixhawk Port Detection --------------------

def find_pixhawk_port():
    """Auto-detect the Pixhawk COM port on Windows."""
    ports = list(serial.tools.list_ports.comports())

    for port in ports:
        print(f"🔍 Checking: {port.device} - {port.description}")
        if any(keyword in port.description.upper() for keyword in ["PX4", "AUTERION", "FTDI", "SILICON", "USB"]):
            print(f"✅ Found Pixhawk: {port.device}")
            return port.device

    print("❌ No valid Pixhawk port found!")
    return None

# -------------------- DroneKit Connection --------------------

def connect_to_pixhawk():
    port = find_pixhawk_port()
    if not port:
        return None

    try:
        print(f"🔌 Connecting to {port} at 115200 baud...")
        vehicle = connect(port, wait_ready=True, baud=115200)
        print("✅ Connected to Pixhawk!")
        return vehicle
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return None

# -------------------- Force Arm --------------------

def force_arm(vehicle):
    """Force arms the drone using MAVLink."""
    print("⚠️ Forcing arm via MAVLink...")

    # Set mode to GUIDED first
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(2)

    vehicle._master.mav.command_long_send(
        vehicle._master.target_system,
        vehicle._master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 21196, 0, 0, 0, 0, 0
    )

    while not vehicle.armed:
        print("⏳ Waiting for drone to arm...")
        time.sleep(1)

    print("✅ Drone is force-armed.")

# -------------------- Takeoff --------------------

def arm_and_takeoff(vehicle, target_altitude):
    """Force arms and takes off to a target altitude."""
    force_arm(vehicle)

    print(f"🚀 Taking off to {target_altitude} meters...")
    vehicle.simple_takeoff(target_altitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt

        if alt is not None:
            print(f"📡 Altitude: {alt:.2f} m")
            if alt >= target_altitude * 0.95:
                print("✅ Target altitude reached!")
                break
        else:
            print("📡 Altitude: waiting for data...")

        time.sleep(1)

# -------------------- Land --------------------

def land(vehicle):
    print("🛬 Landing...")
    vehicle.mode = VehicleMode("LAND")

    while vehicle.armed:
        alt = vehicle.location.global_relative_frame.alt
        print(f"📉 Altitude: {alt:.2f} m")
        time.sleep(1)

    print("✅ Drone has landed and disarmed.")

# -------------------- Main Program --------------------

if __name__ == "__main__":
    vehicle = connect_to_pixhawk()

    if not vehicle:
        print("❌ Could not connect to Pixhawk. Exiting...")
        exit()

    arm_and_takeoff(vehicle, target_altitude=1.5)

    land(vehicle)

    vehicle.close()
    print("🚪 Disconnected from Pixhawk.")