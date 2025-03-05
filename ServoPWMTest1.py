import RPi.GPIO as GPIO
import time

# Setup
servo_pin = 13  # GPIO pin number where the servo is connected
GPIO.setmode(GPIO.BCM)  # Set pin numbering to BCM
GPIO.setup(servo_pin, GPIO.OUT)

# Set the frequency to 50Hz (20ms period)
pwm = GPIO.PWM(servo_pin, 50)  # 50Hz frequency

# Start PWM with a duty cycle of 0 (servo off initially)
pwm.start(0)

# Define a function to set the angle of the servo
def set_servo_angle(angle):
    # The duty cycle for the angle ranges from 2% to 12% (1ms to 2ms pulse width for 0 to 180 degrees)
    duty_cycle = 2.5 + (angle / 180) * 10  # Scale between 2 and 12
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(1.0)  # Let the servo reach the position
    pwm.ChangeDutyCycle(0)  # Stop sending the signal to hold the position

try:
    while True:
        # Example: move to 0 degrees
        set_servo_angle(0)
        time.sleep(1)
        
        # Move to 90 degrees
        set_servo_angle(90)
        time.sleep(1)

        # Move to 180 degrees
        set_servo_angle(180)
        time.sleep(1)

except KeyboardInterrupt:
    # Clean up on CTRL+C exit
    pwm.stop()
    GPIO.cleanup()
