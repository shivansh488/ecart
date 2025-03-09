#!/usr/bin/env python3
"""
anchor_ecart.py

This script implements an anchor device with four UWB modules using the new dw1000.py library.
It processes ranging measurements (simulated here) and controls a dual-channel motor driver (RMCS‑2305)
to drive two motors so that the cart follows the tag.

Wiring (based on RMCS‑2305 datasheet):
  • Motor Driver:
    - SLEEP1  → Pi GPIO 16
    - DIR1    → Pi GPIO 20
    - PWM1    → Pi GPIO 12 (PWM output)
    - SLEEP2  → Pi GPIO 26
    - DIR2    → Pi GPIO 21
    - PWM2    → Pi GPIO 13 (PWM output)
  • UWB Modules: (Each module has its own CS and IRQ, common SPI and common RST)
    - Shared SPI: SCLK (GPIO 11), MOSI (GPIO 10), MISO (GPIO 9), 3.3V, GND.
    - Module1 CS → GPIO 8, IRQ → GPIO 34
    - Module2 CS → GPIO 7, IRQ → GPIO 35
    - Module3 CS → GPIO 25, IRQ → GPIO 36
    - Module4 CS → GPIO 24, IRQ → GPIO 37
    - Common RST for all → GPIO 27
  • Tag (ESP32) wiring remains as before.
"""

import time
import random
import RPi.GPIO as GPIO
from dw1000 import DW1000, DW1000Ranging

# -------------------------
# Motor Driver Configuration (RMCS-2305)
# -------------------------
# Using BCM numbering
SLEEP1_PIN = 16    # Motor 1 enable (active high)
DIR1_PIN   = 20    # Motor 1 direction
PWM1_PIN   = 12    # Motor 1 PWM (speed control)
SLEEP2_PIN = 26    # Motor 2 enable (active high)
DIR2_PIN   = 21    # Motor 2 direction
PWM2_PIN   = 13    # Motor 2 PWM

# Setup motor control pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(SLEEP1_PIN, GPIO.OUT)
GPIO.setup(DIR1_PIN,   GPIO.OUT)
GPIO.setup(PWM1_PIN,   GPIO.OUT)
GPIO.setup(SLEEP2_PIN, GPIO.OUT)
GPIO.setup(DIR2_PIN,   GPIO.OUT)
GPIO.setup(PWM2_PIN,   GPIO.OUT)

# Initialize PWM channels at 1kHz frequency
pwm1 = GPIO.PWM(PWM1_PIN, 1000)
pwm2 = GPIO.PWM(PWM2_PIN, 1000)
pwm1.start(0)  # Start with motor off (0% duty)
pwm2.start(0)

# -------------------------
# UWB Module Configuration
# -------------------------
modules_config = [
    {"cs_pin": 8,  "irq_pin": 34, "rst_pin": 27, "anchor_addr": "84:00:5B:D5:A9:9A:E2:9C", "antenna_delay": 16580},
    {"cs_pin": 7,  "irq_pin": 35, "rst_pin": 27, "anchor_addr": "84:00:5B:D5:A9:9A:E2:9D", "antenna_delay": 16610},
    {"cs_pin": 25, "irq_pin": 36, "rst_pin": 27, "anchor_addr": "84:00:5B:D5:A9:9A:E2:9E", "antenna_delay": 16607},
    {"cs_pin": 24, "irq_pin": 37, "rst_pin": 27, "anchor_addr": "84:00:5B:D5:A9:9A:E2:9F", "antenna_delay": 16611},
]

ranging_modules = []

# For each UWB module, initialize it as an anchor.
for config in modules_config:
    dw = DW1000(spi_channel=0, cs_pin=config["cs_pin"],
                irq_pin=config["irq_pin"], rst_pin=config["rst_pin"])
    dw.reset()
    dw.setAntennaDelay(config["antenna_delay"])
    print(f"Initialized UWB module (CS: {config['cs_pin']}) with antenna delay: {config['antenna_delay']}")
    
    ranging = DW1000Ranging(dw)
    dummy_mode = [0x01, 0x02, 0x03]  # Dummy mode values (adjust if needed)
    ranging.start_as_anchor(config["anchor_addr"], dummy_mode, random_short_address=False)
    ranging_modules.append(ranging)

# -------------------------
# Motor Control Function
# -------------------------
TARGET_DISTANCE = 1.0  # Desired distance from the tag (meters)

def update_motor_control(avg_range):
    # Compute error (positive if tag is further than desired, negative if closer)
    error = avg_range - TARGET_DISTANCE
    
    # Use a simple proportional controller:
    # Scale the error to a duty cycle percentage.
    # For instance, if an error of 0.5 m corresponds to 100% duty cycle.
    scale = 100 / 0.5
    duty_cycle = min(max(abs(error) * scale, 0), 100)
    
    # Set direction:
    # If error > 0, tag is too far: move forward (set DIR low).
    # If error < 0, tag is too close: move backward (set DIR high).
    if error > 0:
        GPIO.output(DIR1_PIN, GPIO.LOW)
        GPIO.output(DIR2_PIN, GPIO.LOW)
    else:
        GPIO.output(DIR1_PIN, GPIO.HIGH)
        GPIO.output(DIR2_PIN, GPIO.HIGH)
    
    # Enable the motors (SLEEP pins active high)
    GPIO.output(SLEEP1_PIN, GPIO.HIGH)
    GPIO.output(SLEEP2_PIN, GPIO.HIGH)
    
    # Update PWM duty cycle to control speed
    pwm1.ChangeDutyCycle(duty_cycle)
    pwm2.ChangeDutyCycle(duty_cycle)
    
    print(f"Avg Range: {avg_range:.2f} m, Error: {error:.2f} m, Duty: {duty_cycle:.1f}%")

# -------------------------
# Dummy Ranging Measurement (Simulation)
# -------------------------
def simulate_range_measurement():
    # In a real application, the ranging measurement comes from processing DW1000 messages.
    # Here, we simulate a measurement around 1.0 m.
    return random.uniform(0.95, 1.05)

# -------------------------
# Main Loop
# -------------------------
try:
    while True:
        # For demonstration, we simulate a range measurement from each module.
        total_range = 0
        for ranging in ranging_modules:
            # Process incoming messages (if any)
            ranging.loop()
            # Use simulated measurement
            total_range += simulate_range_measurement()
        avg_range = total_range / len(ranging_modules)
        
        # Update motor control based on the average measured range.
        update_motor_control(avg_range)
        time.sleep(0.5)
except KeyboardInterrupt:
    print("Exiting anchor application...")
finally:
    # Cleanup: stop PWM and close UWB modules.
    pwm1.stop()
    pwm2.stop()
    for ranging in ranging_modules:
        ranging.dw1000.close()
    GPIO.cleanup()
