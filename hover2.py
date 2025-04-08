import os
import glob
from pymavlink import mavutil
import time
import serial.tools.list_ports

# --------------------------- Pixhawk Connection ---------------------------

def find_pixhawk_port():
    """Finds the correct Pixhawk serial port on Windows (COMx)."""
    ports = list(serial.tools.list_ports.comports())

    for port in ports:
        print(f"Checking: {port.device} - {port.description}")  # Debugging output

        if any(keyword in port.description.upper() for keyword in ["PX4", "AUTERION", "FTDI", "SILICON", "USB"]):
            print(f"✅ Found Pixhawk MAVLink port: {port.device}")
            return port.device  # Returns "COM4" or another valid COM port

    print("❌ No valid Pixhawk MAVLink port found!")
    return None

def connect_to_pixhawk():
    """Connects to Pixhawk via MAVLink."""
    port = find_pixhawk_port()
    if not port:
        return None

    try:
        print(f"🔌 Connecting to {port} at 115200 baud...")
        master = mavutil.mavlink_connection(port, baud=115200)
        master.wait_heartbeat(timeout=5)  # Wait for heartbeat
        print("✅ Connected to Pixhawk!")
        return master
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return None

# --------------------------- Battery Voltage Check ---------------------------

def check_battery_voltage(master):
    """Checks the battery voltage from the Pixhawk and prints raw data."""
    print("🔍 Checking battery voltage...")

    # Request battery status data
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 1, 1
    )

    msg = master.recv_match(type='SYS_STATUS', blocking=True)

    if msg:
        raw_voltage = msg.voltage_battery  # Raw value in millivolts
        voltage = raw_voltage / 1000.0  # Convert to volts
        print(f"🔋 Raw voltage (mV): {raw_voltage}")
        print(f"🔋 Scaled voltage (V): {voltage:.2f}V")

        if voltage < 11.0:
            print("⚠️ WARNING: Battery voltage is LOW!")
        elif voltage < 10.5:
            print("❌ ERROR: Battery voltage TOO LOW for takeoff!")
            exit()
    else:
        print("❌ Failed to retrieve battery voltage!")


# --------------------------- Flight Mode Setup ---------------------------

def set_flight_mode(master, mode="STABILIZE"):
    """Sets the flight mode to STABILIZE (or any other mode)."""
    print(f"📡 Setting mode to {mode}...")

    mode_id = master.mode_mapping().get(mode.upper())

    if mode_id is None:
        print(f"❌ Mode '{mode}' not available!")
        print(f"Available modes: {list(master.mode_mapping().keys())}")
        return

    master.set_mode(mode_id)
    time.sleep(2)

    ack = master.recv_match(type='COMMAND_ACK', blocking=True)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print(f"✅ Mode set to {mode}")
        else:
            print(f"⚠️ Mode change rejected: {ack.result}")


# --------------------------- Arming the Drone ---------------------------

def arm_drone(master):
    """Arms the drone and forces arm if needed."""
    print("🟢 Arming the drone...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(2)

    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    print(f"🔍 Drone system status: {msg.system_status}")

    if msg.system_status != 4:
        print("⚠️ Arming failed. Forcing arm override...")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 21196, 0, 0, 0, 0, 0
        )
        time.sleep(2)

# --------------------------- Takeoff Sequence ---------------------------

def takeoff(master, to = 2000):
    """Takeoff sequence with direct throttle override and acceleration control."""

    # 🚀 Increase throttle before takeoff
    print(f"🚀 Setting throttle to {to}...")
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        0, 0, 0, to,  # Roll, Pitch, Yaw, Throttle
        0, 0, 0, 0
    )
    time.sleep(3)

    # 🔼 Stronger acceleration for lift
    print("🔼 Increasing acceleration for lift-off...")
    for _ in range(5):
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int(0b100111000011),
            0, 0, 0,
            0, 0, 45,
            0, 0, -60,  # Increase acceleration for stronger lift (-40 start)
            0, 0
        )
        time.sleep(1)

    print("✔ Ascension complete!")

# --------------------------- Descend ---------------------------

def descend(master, vz=0.5, duration=5):
    """Command the drone to descend at a set velocity"""
    for _ in range(duration):
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int(0b000011100000),  # Only use velocity
            0, 0, 0,  # Position (ignored)
            0, 0, vz,  # Velocity: Z downward
            0, 0, 0,  # Acceleration (ignored)
            0, 0  # Yaw (ignored)
        )
        print(f"🔽 Descending at {vz} m/s")
        time.sleep(1)

    print("✔ Descent complete!")

# --------------------------- Main Program ---------------------------

if __name__ == "__main__":
    master = connect_to_pixhawk()
    
    if not master:
        print("❌ Could not connect to Pixhawk. Exiting...")
        exit()

    check_battery_voltage(master)

    set_flight_mode(master, "STABILIZED") 

    arm_drone(master)

    takeoff(master)

    descend(master)

    print("🔴 Disarming the drone...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(1)

    print("✅ Flight test complete!")
