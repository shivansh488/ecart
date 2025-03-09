#!/usr/bin/env python3
"""
dw1000.py

A full Python library to interface with the Decawave DW1000 UWB transceiver
on a Raspberry Pi. This library is adapted from the Arduino DW1000 library and
includes:
  - Low-level SPI and GPIO control (reset, read/write registers)
  - Antenna delay configuration
  - Timestamp conversion (DW1000Time)
  - Device management (DW1000Device)
  - A simplified ranging protocol (DW1000Ranging) with basic interrupt and message handling

Dependencies:
  - spidev
  - RPi.GPIO
"""

import spidev
import time
import RPi.GPIO as GPIO
import struct

# -----------------------------------------------------------------------------
# Constants (based on DW1000Constants.h and DW1000Ranging.h)
# -----------------------------------------------------------------------------
# Example register addresses and lengths (adjust these as needed)
DEV_ID     = 0x00
LEN_DEV_ID = 4
EUI        = 0x01
LEN_EUI    = 8
PANADR     = 0x03
LEN_PANADR = 4
SYS_CFG    = 0x04
LEN_SYS_CFG = 4
SYS_CTRL   = 0x0D
LEN_SYS_CTRL = 4
SYS_STATUS = 0x0F
LEN_SYS_STATUS = 5
TX_BUFFER  = 0x09
LEN_TX_BUFFER = 1024
RX_BUFFER  = 0x11
LEN_RX_BUFFER = 1024

# Message types (from the original DW1000Ranging protocol)
POLL        = 0
POLL_ACK    = 1
RANGE       = 2
RANGE_REPORT = 3
RANGE_FAILED = 255
BLINK       = 4
RANGING_INIT = 5

# -----------------------------------------------------------------------------
# DW1000 Class: Low-level interface
# -----------------------------------------------------------------------------
class DW1000:
    def __init__(self, spi_channel=0, cs_pin=8, irq_pin=25, rst_pin=17, spi_bus=0):
        """
        Initialize SPI, GPIO, and internal variables.
          - cs_pin: Chip Select GPIO pin (BCM numbering)
          - irq_pin: Interrupt pin from the DW1000
          - rst_pin: Reset pin to hard reset the DW1000
        """
        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_channel)
        self.spi.max_speed_hz = 16000000  # Adjust based on hardware
        self.spi.mode = 0

        self.cs_pin = cs_pin
        self.irq_pin = irq_pin
        self.rst_pin = rst_pin

        # Initialize GPIO pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.cs_pin, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.irq_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.rst_pin, GPIO.OUT, initial=GPIO.HIGH)

        # Store antenna delay value (for calibration)
        self.antenna_delay = 0

        # Interrupt handling: store a callback if provided
        self.interrupt_callback = None
        GPIO.add_event_detect(self.irq_pin, GPIO.RISING, callback=self._handle_interrupt)

    def _handle_interrupt(self, channel):
        """Internal IRQ handler which calls the user callback if set."""
        if self.interrupt_callback:
            self.interrupt_callback()

    def set_interrupt_callback(self, callback):
        """Set a callback function to be called when an IRQ occurs."""
        self.interrupt_callback = callback

    def reset(self):
        """Perform a hardware reset via the RST pin."""
        GPIO.output(self.rst_pin, GPIO.LOW)
        time.sleep(0.01)
        GPIO.output(self.rst_pin, GPIO.HIGH)
        time.sleep(0.01)

    def soft_reset(self):
        """
        Perform a software reset.
        (This is a placeholder – in a complete implementation, write to SYS_CTRL.)
        """
        self.write_bytes(SYS_CTRL, [0x01])
        time.sleep(0.01)

    def write_bytes(self, reg, data):
        """
        Write a sequence of bytes to the given register.
        Assumes a one-byte register address.
        """
        GPIO.output(self.cs_pin, GPIO.LOW)
        self.spi.xfer2([reg] + list(data))
        GPIO.output(self.cs_pin, GPIO.HIGH)

    def read_bytes(self, reg, length):
        """
        Read a sequence of bytes from the given register.
        Returns a list of byte values.
        """
        GPIO.output(self.cs_pin, GPIO.LOW)
        resp = self.spi.xfer2([reg] + [0]*length)
        GPIO.output(self.cs_pin, GPIO.HIGH)
        return resp[1:]  # Skip the first dummy byte

    def setAntennaDelay(self, delay):
        """
        Set the antenna delay for calibration.
        In a complete implementation, this writes to a specific register.
        Here we store the value and print it.
        """
        self.antenna_delay = delay
        # For example, write to a register (placeholder code):
        # self.write_bytes(ANTENNA_DELAY_REG, list(struct.pack('<H', delay)))
        print(f"Set antenna delay to {delay}")

    def close(self):
        """Close the SPI connection and clean up the GPIO pins used."""
        self.spi.close()
        GPIO.cleanup([self.cs_pin, self.irq_pin, self.rst_pin])

# -----------------------------------------------------------------------------
# DW1000Time Class: Timestamp Handling
# -----------------------------------------------------------------------------
class DW1000Time:
    """
    Handles DW1000 timestamps (40-bit) with ~15.65ps resolution.
    """
    TIME_RES = 0.000015650040064103  # microseconds per timestamp unit
    TIME_OVERFLOW = 0x10000000000     # Overflow value (40 bits)

    def __init__(self, timestamp=0):
        self.timestamp = timestamp

    def set_time_us(self, time_us):
        """Set timestamp based on microseconds."""
        self.timestamp = int(time_us / DW1000Time.TIME_RES)

    def get_time_us(self):
        """Get the time in microseconds."""
        return (self.timestamp % DW1000Time.TIME_OVERFLOW) * DW1000Time.TIME_RES

    def get_as_meters(self):
        """
        Convert the time-of-flight to a distance in meters.
        (Uses a conversion factor from the original library.)
        """
        DISTANCE_OF_RADIO = 0.0046917639786159
        return (self.timestamp % DW1000Time.TIME_OVERFLOW) * DISTANCE_OF_RADIO

    def to_bytes(self):
        """Return the timestamp as a 5-byte little-endian array."""
        return self.timestamp.to_bytes(5, byteorder='little')

    @classmethod
    def from_bytes(cls, data):
        """Create a DW1000Time instance from a 5-byte little-endian array."""
        timestamp = int.from_bytes(data, byteorder='little')
        return cls(timestamp)

# -----------------------------------------------------------------------------
# DW1000Device Class: Represents a remote device
# -----------------------------------------------------------------------------
class DW1000Device:
    def __init__(self, address=None, short_address=None):
        """
        Create a device instance with an 8-byte unique address and a 2-byte short address.
        """
        if address is None:
            self.address = bytearray(8)
        else:
            self.address = bytearray(address)
        if short_address is None:
            self.short_address = bytearray(2)
        else:
            self.short_address = bytearray(short_address)
        self.range = 0.0      # Measured range in meters
        self.rx_power = 0.0   # Received power (arbitrary units)
        self.fp_power = 0.0   # First path power (arbitrary units)
        self.quality = 0.0    # Quality metric

        # Timestamps for ranging events (optional)
        self.time_poll_sent = None
        self.time_poll_received = None
        self.time_poll_ack_sent = None
        self.time_poll_ack_received = None
        self.time_range_sent = None
        self.time_range_received = None

    def set_range(self, range_m):
        self.range = range_m

    def get_range(self):
        return self.range

# -----------------------------------------------------------------------------
# DW1000Ranging Class: Ranging Protocol Implementation
# -----------------------------------------------------------------------------
class DW1000Ranging:
    """
    Implements a simplified ranging protocol.
    Handles network device management, message sending/receiving,
    and basic ranging calculations.
    """
    def __init__(self, dw1000):
        self.dw1000 = dw1000
        self.devices = []           # List of DW1000Device objects in the network
        self.reply_delay_us = 7000  # Default reply delay in microseconds

        # Handlers (callbacks) for events
        self.new_range_handler = None
        self.new_device_handler = None
        self.inactive_device_handler = None

        # Simulated message buffer for demonstration purposes.
        self.message_buffer = []
        # Device type: 'anchor' or 'tag'
        self.device_type = None

    @staticmethod
    def initCommunication(rst_pin, cs_pin, irq_pin):
        """
        Initialize communication.
        (Provided for compatibility – actual initialization is done in DW1000.__init__.)
        """
        pass

    def attachNewRange(self, handler):
        """Attach a callback for when a new range measurement is available."""
        self.new_range_handler = handler

    def attachNewDevice(self, handler):
        """Attach a callback for when a new device is discovered."""
        self.new_device_handler = handler

    def attachInactiveDevice(self, handler):
        """Attach a callback for when a device becomes inactive."""
        self.inactive_device_handler = handler

    def add_device(self, device):
        """
        Add a new device to the network if not already present.
        """
        for d in self.devices:
            if d.address == device.address:
                return False
        self.devices.append(device)
        if self.new_device_handler:
            self.new_device_handler(device)
        return True

    def remove_device(self, device):
        """
        Remove a device from the network.
        """
        self.devices = [d for d in self.devices if d.address != device.address]
        if self.inactive_device_handler:
            self.inactive_device_handler(device)

    def configure_network(self, device_address, network_id, mode):
        """
        Configure network parameters.
        In a full implementation, this would write to registers (e.g., PANADR).
        Here, we simply print the configuration.
        """
        print(f"Configuring network: device_address={device_address}, network_id={network_id}, mode={mode}")

    def start_as_anchor(self, address, mode, random_short_address=True):
        """
        Start the device as an anchor.
        Sets the device address and network configuration.
        """
        addr_bytes = bytearray.fromhex(address.replace(":", ""))
        if random_short_address:
            import random
            short_addr = bytearray([random.randint(0, 255), random.randint(0, 255)])
        else:
            short_addr = addr_bytes[:2]
        self.my_address = addr_bytes
        self.my_short_address = short_addr
        self.device_type = 'anchor'
        self.configure_network(int.from_bytes(short_addr, 'big'), 0xDECA, mode)
        print("Started as Anchor with address:", address)

    def start_as_tag(self, address, mode, random_short_address=True):
        """
        Start the device as a tag.
        """
        addr_bytes = bytearray.fromhex(address.replace(":", ""))
        if random_short_address:
            import random
            short_addr = bytearray([random.randint(0, 255), random.randint(0, 255)])
        else:
            short_addr = addr_bytes[:2]
        self.my_address = addr_bytes
        self.my_short_address = short_addr
        self.device_type = 'tag'
        self.configure_network(int.from_bytes(short_addr, 'big'), 0xDECA, mode)
        print("Started as Tag with address:", address)

    def send_message(self, msg_type, payload):
        """
        Send a message of the given type with payload.
        In a real implementation, this writes to the DW1000 transmit buffer.
        Here, we simulate by appending to a message buffer.
        """
        msg = bytes([msg_type]) + payload
        print(f"Sending message (type {msg_type}): {msg}")
        self.message_buffer.append(msg)

    def receive_message(self):
        """
        Check if a message has been received.
        In a real system, this would be triggered by an interrupt and read from registers.
        Here, we simulate by checking our message_buffer.
        """
        if self.message_buffer:
            return self.message_buffer.pop(0)
        return None

    def process_message(self, msg):
        """
        Process a received message by checking its type and acting accordingly.
        """
        if not msg:
            return
        msg_type = msg[0]
        payload = msg[1:]
        if msg_type == POLL and self.device_type == 'anchor':
            print("Received POLL message")
            # Upon receiving a POLL, the anchor sends a POLL_ACK.
            self.send_message(POLL_ACK, b'')
        elif msg_type == RANGE and self.device_type == 'anchor':
            print("Received RANGE message")
            # Simulate a range calculation; in a full system, timestamps are processed here.
            dummy_range = 1.0  # dummy value in meters
            if self.devices:
                self.devices[0].set_range(dummy_range)
                if self.new_range_handler:
                    self.new_range_handler()
        # Additional message types (RANGE_REPORT, BLINK, etc.) can be handled similarly.

    def loop(self):
        """
        Main loop for processing ranging messages.
        In a complete implementation, this method would be called repeatedly.
        """
        msg = self.receive_message()
        if msg:
            self.process_message(msg)
        # Additional periodic tasks could be added here.

# End of dw1000.py
