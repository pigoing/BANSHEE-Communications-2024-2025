import os
import glob
from pymavlink import mavutil
import time

import serial.tools.list_ports


def find_pixhawk_port():
    """Finds the correct Pixhawk serial port on Windows (COMx)."""
    ports = list(serial.tools.list_ports.comports())

    for port in ports:
        print(f"Checking: {port.device} - {port.description}")  # Debugging output

        # Check for common Pixhawk keywords
        if any(keyword in port.description.upper() for keyword in ["PX4", "AUTERION", "FTDI", "SILICON", "USB"]):
            print(f"✅ Found Pixhawk MAVLink port: {port.device}")
            return port.device  # Returns "COM4" or another valid COM port

    print("❌ No valid Pixhawk MAVLink port found!")
    return None

'''def find_pixhawk_port():
    """Finds the correct Pixhawk serial port (if00)."""
    serial_ports = glob.glob('/dev/serial/-id/*')

    for port in serial_ports:
        if "PX4" in port or "AUTERION" in port:
            if port.endswith("-if00"):  # Select only the main MAVLink interface
                print(f"Found Pixhawk MAVLink port: {port}")
                return port

    print("No valid Pixhawk MAVLink port found!")
    return None'''


def connect_to_pixhawk():
    """Connects to Pixhawk via MAVLink."""
    port = find_pixhawk_port()
    if not port:
        return None

    try:
        # Ensure COMx format is used on Windows
        connection_string = port if "COM" in port else f"COM{port}"
        print(f"🔌 Connecting to {connection_string} at 115200 baud...")

        master = mavutil.mavlink_connection(connection_string, baud=115200)
        master.wait_heartbeat(timeout=5)  # Wait for Pixhawk's heartbeat
        print("✅ Connected to Pixhawk!")
        return master
    except Exception as e:
        print(f"❌ Failed to connect: {e}")
        return None
'''def connect_to_pixhawk():
    """Connects to Pixhawk via MAVLink."""
    port = find_pixhawk_port()
    if not port:
        return None

    try:
        master = mavutil.mavlink_connection(port, baud=115200)
        master.wait_heartbeat(timeout=5)
        print("Connected to Pixhawk!")
        return master
    except Exception as e:
        print(f"Failed to connect: {e}")
        return None'''

def ascend_and_hover(master, az=25, ascend_duration=4, hover_duration=5):
    """Command the drone to ascend and then hover in place."""
    
    # Ascend
    for _ in range(ascend_duration):
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Local NED frame
            int(0b100111000011),  # Only use velocity
            0, 0, 0,  # Position (ignored)
            0, 0, 0,  # Velocity: Upward (negative Z in NED)
            0, 0, -az,  # Acceleration (ignored)
            0, 0  # Yaw (ignored)
        )
        print(f"🔼 Ascending at {az} m/s")
        time.sleep(1)

    print("✔ Ascension complete! Transitioning to hover...")

    # Hover
    for _ in range(hover_duration):
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Local NED frame
            int(0b110111000111),  # Only use velocity
            0, 0, 0,  # Position (ignored)
            0, 0, 0,  # Velocity: Zero in all directions
            0, 0, 0,  # Acceleration (ignored)
            0, 0  # Yaw (ignored)
        )
        print("🛑 Hovering in place")
        time.sleep(1)

    print("✔ Hover complete!")


def descend(master, vz=0.5, duration=5):
    """Command the drone to descend at a set velocity"""
    for _ in range(duration):
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Local NED frame
            int(0b110111000111),  # Only use velocity
            0, 0, 0,  # Position (ignored)
            0, 0, vz,  # Velocity: Only Z downward
            0, 0, 0,  # Acceleration (ignored)
            0, 0  # Yaw (ignored)s
        )
        print(f"🔽 Descending at {vz} m/s")
        time.sleep(1)

    print("✔ Descent complete!")

# Run the connection test
if __name__ == "__main__":
    master = connect_to_pixhawk()
    if master:
        master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

        time.sleep(1)

        ascend_and_hover(master)

        descend(master)

        time.sleep(1)

        master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)