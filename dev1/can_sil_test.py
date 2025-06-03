#!/usr/bin/env python3
"""
Software-in-the-Loop (SIL) Test Suite for Cascadia Motor Controller CAN Messages
Sends realistic dummy CAN messages over socketcan to test Arduino CAN decoder
"""

import can
import time
import struct
import threading
import random
import math
from typing import Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum
import numpy as np

class TestMode(Enum):
    NORMAL_OPERATION = "normal"
    FAULT_INJECTION = "fault"
    STRESS_TEST = "stress"
    TEMPERATURE_SWEEP = "temp_sweep"
    VOLTAGE_RAMP = "voltage_ramp"
    MOTOR_ACCELERATION = "motor_accel"
    SHUTDOWN_SEQUENCE = "shutdown"
    CUSTOM = "custom"

@dataclass
class CANConfig:
    """CAN bus configuration matching Arduino code"""
    BASE_ID = 0x0A0
    FAULT_ID = 0x0AB
    CHANNEL = 'can0'  # Virtual CAN interface, change to can0 for real hardware
    BITRATE = 500000   # 500 kbps

class MessageEncoder:
    """Encodes values according to Cascadia protocol"""
    
    @staticmethod
    def encode_temperature(temp_celsius: float) -> bytes:
        """Encode temperature as int16 * 10"""
        value = int(temp_celsius * 10)
        return struct.pack('<h', value)  # Little endian int16
    
    @staticmethod
    def encode_high_voltage(voltage: float) -> bytes:
        """Encode high voltage as int16 * 10"""
        value = int(voltage * 10)
        return struct.pack('<h', value)
    
    @staticmethod
    def encode_low_voltage(voltage: float) -> bytes:
        """Encode low voltage as int16 * 100"""
        value = int(voltage * 100)
        return struct.pack('<h', value)
    
    @staticmethod
    def encode_current(current: float) -> bytes:
        """Encode current as int16 * 10"""
        value = int(current * 10)
        return struct.pack('<h', value)
    
    @staticmethod
    def encode_torque(torque: float) -> bytes:
        """Encode torque as int16 * 10"""
        value = int(torque * 10)
        return struct.pack('<h', value)
    
    @staticmethod
    def encode_angular_velocity(rpm: int) -> bytes:
        """Encode RPM as int16"""
        return struct.pack('<h', rpm)
    
    @staticmethod
    def encode_frequency(freq: float) -> bytes:
        """Encode frequency as int16 * 10"""
        value = int(freq * 10)
        return struct.pack('<h', value)

class VehicleSimulator:
    """Simulates realistic vehicle parameters"""
    
    def __init__(self):
        self.time_start = time.time()
        self.motor_speed = 0  # RPM
        self.torque_command = 0  # Nm
        self.dc_voltage = 400  # V
        self.temperatures = {
            'module_a': 25.0,
            'module_b': 25.0, 
            'module_c': 25.0,
            'gate_driver': 30.0,
            'control_board': 35.0,
            'coolant': 40.0,
            'motor': 50.0
        }
        self.vsm_state = 0
        self.inverter_state = 0
        
    def update(self, dt: float, mode: TestMode):
        """Update simulation parameters based on test mode"""
        current_time = time.time() - self.time_start
        
        if mode == TestMode.NORMAL_OPERATION:
            self._normal_operation(current_time, dt)
        elif mode == TestMode.MOTOR_ACCELERATION:
            self._motor_acceleration(current_time, dt)
        elif mode == TestMode.TEMPERATURE_SWEEP:
            self._temperature_sweep(current_time, dt)
        elif mode == TestMode.VOLTAGE_RAMP:
            self._voltage_ramp(current_time, dt)
        elif mode == TestMode.STRESS_TEST:
            self._stress_test(current_time, dt)
        elif mode == TestMode.SHUTDOWN_SEQUENCE:
            self._shutdown_sequence(current_time, dt)
    
    def _normal_operation(self, t: float, dt: float):
        """Simulate normal vehicle operation"""
        # Sinusoidal speed variation
        self.motor_speed = int(1500 + 500 * math.sin(t * 0.1))
        self.torque_command = 100 + 50 * math.sin(t * 0.2)
        
        # Temperature slowly increases with load
        base_temp_rise = abs(self.torque_command) / 10
        for key in self.temperatures:
            if 'module' in key:
                self.temperatures[key] = 25 + base_temp_rise + random.uniform(-2, 2)
        
        self.vsm_state = 6  # Motor Running State
        self.inverter_state = 3  # Closed Loop State
    
    def _motor_acceleration(self, t: float, dt: float):
        """Simulate motor acceleration from 0 to max speed"""
        max_speed = 3000
        accel_time = 20  # seconds
        
        if t < accel_time:
            self.motor_speed = int((t / accel_time) * max_speed)
            self.torque_command = 200 * (1 - t / accel_time)  # High torque at start
        else:
            self.motor_speed = max_speed
            self.torque_command = 50  # Steady state torque
        
        # Temperature rises with acceleration
        temp_factor = min(t / 10, 2.0)
        for key in self.temperatures:
            self.temperatures[key] = 25 + 20 * temp_factor
    
    def _temperature_sweep(self, t: float, dt: float):
        """Sweep temperatures through operating range"""
        temp_cycle = 60  # 60 second cycle
        phase = (t % temp_cycle) / temp_cycle * 2 * math.pi
        
        base_temp = 50 + 40 * math.sin(phase)
        for key in self.temperatures:
            offset = random.uniform(-5, 5)
            self.temperatures[key] = base_temp + offset
        
        self.motor_speed = 2000
        self.torque_command = 150
    
    def _voltage_ramp(self, t: float, dt: float):
        """Ramp voltage up and down"""
        ramp_time = 30
        phase = (t % ramp_time) / ramp_time
        
        if phase < 0.5:
            self.dc_voltage = 300 + 200 * (phase * 2)  # 300V to 500V
        else:
            self.dc_voltage = 500 - 200 * ((phase - 0.5) * 2)  # 500V to 300V
        
        self.motor_speed = 1800
        self.torque_command = 120
    
    def _stress_test(self, t: float, dt: float):
        """High frequency random variations"""
        self.motor_speed = int(2000 + 1000 * random.uniform(-1, 1))
        self.torque_command = 150 + 100 * random.uniform(-1, 1)
        
        for key in self.temperatures:
            self.temperatures[key] += random.uniform(-1, 1)
            self.temperatures[key] = max(20, min(150, self.temperatures[key]))
    
    def _shutdown_sequence(self, t: float, dt: float):
        """Simulate vehicle shutdown sequence"""
        if t < 5:
            self.vsm_state = 6  # Motor Running
            self.motor_speed = int(2000 * (1 - t/5))
        elif t < 10:
            self.vsm_state = 4  # VSM Wait State
            self.motor_speed = 0
        elif t < 15:
            self.vsm_state = 14  # Shutdown in Process
        else:
            self.vsm_state = 15  # Recycle Power State

class CANSILTester:
    """Main SIL test controller"""
    
    def __init__(self, channel: str = CANConfig.CHANNEL):
        self.config = CANConfig()
        self.config.CHANNEL = channel
        self.encoder = MessageEncoder()
        self.simulator = VehicleSimulator()
        self.bus = None
        self.running = False
        self.thread = None
        self.current_mode = TestMode.NORMAL_OPERATION
        self.send_rate = 10  # Hz
        self.fault_codes = [0x0000, 0x0000, 0x0000, 0x0000]  # POST Lo, POST Hi, Run Lo, Run Hi
        
    def connect(self) -> bool:
        """Connect to CAN bus"""
        try:
            self.bus = can.interface.Bus(
                channel=self.config.CHANNEL,
                bustype='socketcan',
                bitrate=self.config.BITRATE
            )
            print(f"Connected to CAN bus: {self.config.CHANNEL}")
            return True
        except Exception as e:
            print(f"Failed to connect to CAN bus: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from CAN bus"""
        if self.bus:
            self.bus.shutdown()
            self.bus = None
            print("Disconnected from CAN bus")
    
    def set_mode(self, mode: TestMode):
        """Set the current test mode"""
        self.current_mode = mode
        print(f"Test mode set to: {mode.value}")
    
    def set_send_rate(self, rate_hz: float):
        """Set message send rate in Hz"""
        self.send_rate = rate_hz
        print(f"Send rate set to: {rate_hz} Hz")
    
    def inject_fault(self, fault_type: str, fault_bit: int):
        """Inject specific fault codes"""
        if fault_type == "post_lo":
            self.fault_codes[0] |= (1 << fault_bit)
        elif fault_type == "post_hi":
            self.fault_codes[1] |= (1 << fault_bit)
        elif fault_type == "run_lo":
            self.fault_codes[2] |= (1 << fault_bit)
        elif fault_type == "run_hi":
            self.fault_codes[3] |= (1 << fault_bit)
        print(f"Injected fault: {fault_type} bit {fault_bit}")
    
    def clear_faults(self):
        """Clear all fault codes"""
        self.fault_codes = [0x0000, 0x0000, 0x0000, 0x0000]
        print("All faults cleared")
    
    def _create_temperatures_1_message(self) -> can.Message:
        """Create Temperatures #1 message (0x0A0)"""
        data = bytearray(8)
        data[0:2] = self.encoder.encode_temperature(self.simulator.temperatures['module_a'])
        data[2:4] = self.encoder.encode_temperature(self.simulator.temperatures['module_b'])
        data[4:6] = self.encoder.encode_temperature(self.simulator.temperatures['module_c'])
        data[6:8] = self.encoder.encode_temperature(self.simulator.temperatures['gate_driver'])
        
        return can.Message(arbitration_id=self.config.BASE_ID + 0x00, data=data, is_extended_id=False)
    
    def _create_temperatures_2_message(self) -> can.Message:
        """Create Temperatures #2 message (0x0A1)"""
        data = bytearray(8)
        data[0:2] = self.encoder.encode_temperature(self.simulator.temperatures['control_board'])
        data[2:4] = self.encoder.encode_temperature(25.0)  # RTD1
        data[4:6] = self.encoder.encode_temperature(30.0)  # RTD2
        data[6:8] = self.encoder.encode_temperature(35.0)  # RTD3
        
        return can.Message(arbitration_id=self.config.BASE_ID + 0x01, data=data, is_extended_id=False)
    
    def _create_temperatures_3_message(self) -> can.Message:
        """Create Temperatures #3 & Torque Shudder message (0x0A2)"""
        data = bytearray(8)
        data[0:2] = self.encoder.encode_temperature(self.simulator.temperatures['coolant'])
        data[2:4] = self.encoder.encode_temperature(60.0)  # Hotspot
        data[4:6] = self.encoder.encode_temperature(self.simulator.temperatures['motor'])
        data[6:8] = self.encoder.encode_torque(5.0)  # Torque shudder
        
        return can.Message(arbitration_id=self.config.BASE_ID + 0x02, data=data, is_extended_id=False)
    
    def _create_voltage_message(self) -> can.Message:
        """Create Voltage Information message (0x0A7)"""
        data = bytearray(8)
        data[0:2] = self.encoder.encode_high_voltage(self.simulator.dc_voltage)
        data[2:4] = self.encoder.encode_high_voltage(self.simulator.dc_voltage * 0.95)  # Output voltage
        data[4:6] = self.encoder.encode_high_voltage(self.simulator.dc_voltage * 0.6)  # VAB/Vd
        data[6:8] = self.encoder.encode_high_voltage(self.simulator.dc_voltage * 0.4)  # VBC/Vq
        
        return can.Message(arbitration_id=self.config.BASE_ID + 0x07, data=data, is_extended_id=False)
    
    def _create_current_message(self) -> can.Message:
        """Create Current Information message (0x0A6)"""
        base_current = abs(self.simulator.torque_command) / 10  # Simple torque to current mapping
        data = bytearray(8)
        data[0:2] = self.encoder.encode_current(base_current * 1.1)  # Phase A
        data[2:4] = self.encoder.encode_current(base_current * 0.9)  # Phase B
        data[4:6] = self.encoder.encode_current(base_current * 1.0)  # Phase C
        data[6:8] = self.encoder.encode_current(base_current * 0.8)  # DC Bus
        
        return can.Message(arbitration_id=self.config.BASE_ID + 0x06, data=data, is_extended_id=False)
    
    def _create_motor_position_message(self) -> can.Message:
        """Create Motor Position message (0x0A5)"""
        angle = (time.time() * self.simulator.motor_speed / 60 * 360) % 360  # Calculate angle from speed
        frequency = self.simulator.motor_speed / 60 * 2  # Assume 2 pole pairs
        
        data = bytearray(8)
        data[0:2] = self.encoder.encode_temperature(angle)  # Reuse temperature encoding for angle
        data[2:4] = self.encoder.encode_angular_velocity(self.simulator.motor_speed)
        data[4:6] = self.encoder.encode_frequency(frequency)
        data[6:8] = self.encoder.encode_temperature(0.5)  # Delta resolver
        
        return can.Message(arbitration_id=self.config.BASE_ID + 0x05, data=data, is_extended_id=False)
    
    def _create_torque_timer_message(self) -> can.Message:
        """Create Torque & Timer message (0x0AC)"""
        feedback_torque = self.simulator.torque_command * random.uniform(0.95, 1.05)  # Add noise
        power_timer = int((time.time() - self.simulator.time_start) / 0.003)  # Convert to timer units
        
        data = bytearray(8)
        data[0:2] = self.encoder.encode_torque(self.simulator.torque_command)
        data[2:4] = self.encoder.encode_torque(feedback_torque)
        data[4:8] = struct.pack('<L', power_timer)  # 32-bit timer value
        
        return can.Message(arbitration_id=self.config.BASE_ID + 0x0C, data=data, is_extended_id=False)
    
    def _create_internal_states_message(self) -> can.Message:
        """Create Internal States message (0x0AA)"""
        data = bytearray(8)
        data[0] = self.simulator.vsm_state
        data[1] = 8  # PWM frequency in kHz
        data[2] = self.simulator.inverter_state
        data[3] = 0x0F  # Relay state
        data[4] = 0x00  # Reserved
        data[5] = 0x00  # Reserved
        data[6] = 0x00  # Reserved
        data[7] = 0x00  # Reserved
        
        return can.Message(arbitration_id=self.config.BASE_ID + 0x0A, data=data, is_extended_id=False)
    
    def _create_fault_message(self) -> can.Message:
        """Create Fault Codes message (0x0AB)"""
        data = bytearray(8)
        data[0:2] = struct.pack('<H', self.fault_codes[0])  # POST Fault Lo
        data[2:4] = struct.pack('<H', self.fault_codes[1])  # POST Fault Hi
        data[4:6] = struct.pack('<H', self.fault_codes[2])  # Run Fault Lo
        data[6:8] = struct.pack('<H', self.fault_codes[3])  # Run Fault Hi
        
        return can.Message(arbitration_id=self.config.FAULT_ID, data=data, is_extended_id=False)
    
    def _send_message_cycle(self):
        """Send one complete cycle of all messages"""
        if not self.bus:
            return
        
        messages = [
            self._create_temperatures_1_message(),
            self._create_temperatures_2_message(),
            self._create_temperatures_3_message(),
            self._create_voltage_message(),
            self._create_current_message(),
            self._create_motor_position_message(),
            self._create_torque_timer_message(),
            self._create_internal_states_message(),
        ]
        
        # Add fault message if faults are present or in fault injection mode
        if any(self.fault_codes) or self.current_mode == TestMode.FAULT_INJECTION:
            if self.current_mode == TestMode.FAULT_INJECTION:
                # Inject random faults for testing
                self.fault_codes[2] = random.randint(0, 0xFFFF) if random.random() < 0.1 else 0
            messages.append(self._create_fault_message())
        
        # Send all messages
        for msg in messages:
            try:
                self.bus.send(msg)
            except Exception as e:
                print(f"Error sending message {hex(msg.arbitration_id)}: {e}")
    
    def _run_loop(self):
        """Main execution loop"""
        dt = 1.0 / self.send_rate
        
        while self.running:
            start_time = time.time()
            
            # Update simulation
            self.simulator.update(dt, self.current_mode)
            
            # Send messages
            self._send_message_cycle()
            
            # Maintain send rate
            elapsed = time.time() - start_time
            sleep_time = max(0, dt - elapsed)
            time.sleep(sleep_time)
    
    def start(self):
        """Start sending messages"""
        if not self.bus:
            print("Not connected to CAN bus")
            return False
        
        if self.running:
            print("Already running")
            return False
        
        self.running = True
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        print(f"Started sending messages at {self.send_rate} Hz in {self.current_mode.value} mode")
        return True
    
    def stop(self):
        """Stop sending messages"""
        if not self.running:
            print("Not running")
            return
        
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        print("Stopped sending messages")
    
    def status(self):
        """Print current status"""
        print(f"\n--- CAN SIL Tester Status ---")
        print(f"Connected: {'Yes' if self.bus else 'No'}")
        print(f"Running: {'Yes' if self.running else 'No'}")
        print(f"Mode: {self.current_mode.value}")
        print(f"Send Rate: {self.send_rate} Hz")
        print(f"Channel: {self.config.CHANNEL}")
        print(f"Motor Speed: {self.simulator.motor_speed} RPM")
        print(f"Torque Command: {self.simulator.torque_command:.1f} Nm")
        print(f"DC Voltage: {self.simulator.dc_voltage:.1f} V")
        print(f"VSM State: {self.simulator.vsm_state}")
        print(f"Fault Codes: {[hex(f) for f in self.fault_codes]}")

# Convenience functions for Jupyter Lab usage
def create_tester(channel: str = "vcan0") -> CANSILTester:
    """Create and return a new CAN SIL tester instance"""
    return CANSILTester(channel)

def setup_virtual_can():
    """Instructions for setting up virtual CAN interface"""
    print("""
    To set up virtual CAN interface (run in terminal):
    
    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan
    sudo ip link set up vcan0
    
    To monitor CAN traffic:
    candump vcan0
    
    To use real CAN hardware, change channel to 'can0' or appropriate interface.
    """)

# Example usage
if __name__ == "__main__":
    # This section runs when executed as a script
    setup_virtual_can()
    
    tester = create_tester()
    
    if tester.connect():
        print("\nStarting normal operation test...")
        tester.set_mode(TestMode.NORMAL_OPERATION)
        tester.start()
        
        try:
            time.sleep(10)
            print("\nSwitching to motor acceleration test...")
            tester.set_mode(TestMode.MOTOR_ACCELERATION)
            time.sleep(10)
            
            print("\nInjecting fault...")
            tester.inject_fault("run_lo", 0)  # Hardware gate fault
            tester.set_mode(TestMode.FAULT_INJECTION)
            time.sleep(5)
            
            tester.status()
            
        except KeyboardInterrupt:
            print("\nStopping test...")
        finally:
            tester.stop()
            tester.disconnect()