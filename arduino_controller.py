"""
Arduino Communication Module
Handles serial communication with Arduino for motor control
"""

import serial
import serial.tools.list_ports
import threading
import queue
import time
import json
from typing import Optional, List, Tuple, Dict
from dataclasses import dataclass
from enum import Enum
import struct

class CommandType(Enum):
    MOVE = 'M'
    SETTINGS = 'S'
    RESET = 'R'
    QUERY = 'Q'
    EMERGENCY_STOP = 'E'
    HOME = 'H'

@dataclass
class ArduinoFeedback:
    """Feedback data from Arduino"""
    current_angle: float
    target_angle: float
    current_speed: float
    is_moving: bool
    timestamp: int
    raw_data: str

@dataclass
class ArduinoDevice:
    """Arduino device information"""
    port: str
    description: str
    hwid: str
    vid: int
    pid: int
    serial_number: str

class ArduinoController:
    """Manages communication with Arduino stepper controller"""
    
    def __init__(self, config: dict):
        self.config = config
        self.serial_port = None
        self.connected = False
        
        # Communication threads and queues
        self.command_queue = queue.Queue()
        self.feedback_queue = queue.Queue(maxsize=100)
        self.read_thread = None
        self.write_thread = None
        self.running = False
        
        # Current state
        self.current_position = 0.0
        self.target_position = 0.0
        self.is_moving = False
        self.last_feedback = None
        self.last_feedback_time = 0
        
        # Performance tracking
        self.command_count = 0
        self.feedback_count = 0
        self.errors = 0
        self.latency = 0
        
        # Callbacks
        self.feedback_callback = None
        self.error_callback = None
        
        # Thread safety
        self.lock = threading.Lock()
    
    @staticmethod
    def list_devices() -> List[ArduinoDevice]:
        """List all available Arduino devices"""
        devices = []
        ports = serial.tools.list_ports.comports()
        
        for port in ports:
            # Check if it's likely an Arduino
            if ('Arduino' in port.description or 
                'USB' in port.description or
                'CH340' in port.description or
                'FTDI' in port.description or
                port.vid == 0x2341):  # Arduino vendor ID
                
                device = ArduinoDevice(
                    port=port.device,
                    description=port.description,
                    hwid=port.hwid,
                    vid=port.vid or 0,
                    pid=port.pid or 0,
                    serial_number=port.serial_number or ""
                )
                devices.append(device)
        
        return devices
    
    def connect(self, port: Optional[str] = None, baudrate: Optional[int] = None) -> bool:
        """Connect to Arduino on specified port"""
        if self.connected:
            self.disconnect()
        
        port = port or self.config['port']
        baudrate = baudrate or self.config['baudrate']
        
        try:
            # Open serial connection
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=self.config.get('timeout', 0.1),
                write_timeout=0.5,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            # Clear buffers
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            # Wait for Arduino to initialize
            time.sleep(2)
            
            # Check for READY message
            start_time = time.time()
            while time.time() - start_time < 5:
                if self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if 'READY' in line:
                        break
            
            # Start communication threads
            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.write_thread = threading.Thread(target=self._write_loop, daemon=True)
            self.read_thread.start()
            self.write_thread.start()
            
            # Send initial configuration
            self._send_settings()
            
            # Query current status
            self.send_command(CommandType.QUERY)
            
            self.connected = True
            print(f"Connected to Arduino on {port}")
            return True
            
        except Exception as e:
            print(f"Failed to connect to Arduino: {e}")
            if self.serial_port:
                self.serial_port.close()
            return False
    
    def disconnect(self):
        """Disconnect from Arduino"""
        self.running = False
        
        # Wait for threads to finish
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=1)
        if self.write_thread and self.write_thread.is_alive():
            self.write_thread.join(timeout=1)
        
        # Close serial port
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        
        self.connected = False
        print("Disconnected from Arduino")
    
    def move_to_angle(self, angle: float):
        """Command Arduino to move to specified angle"""
        if not self.connected:
            return False
        
        # Limit angle to valid range
        angle = max(-180, min(180, angle))
        
        # Send move command
        command = f"{CommandType.MOVE.value},{angle:.2f}"
        self.command_queue.put(command)
        self.target_position = angle
        return True
    
    def send_command(self, command_type: CommandType, params: Optional[List] = None):
        """Send a command to Arduino"""
        if not self.connected:
            return False
        
        if params:
            param_str = ','.join(str(p) for p in params)
            command = f"{command_type.value},{param_str}"
        else:
            command = command_type.value
        
        self.command_queue.put(command)
        return True
    
    def emergency_stop(self):
        """Send emergency stop command"""
        if not self.connected:
            return False
        
        # Clear command queue
        while not self.command_queue.empty():
            try:
                self.command_queue.get_nowait()
            except queue.Empty:
                break
        
        # Send stop command with priority
        self.command_queue.put(CommandType.EMERGENCY_STOP.value)
        return True
    
    def reset_position(self):
        """Reset Arduino position to zero"""
        return self.send_command(CommandType.RESET)
    
    def home(self):
        """Move to home position (0 degrees)"""
        return self.send_command(CommandType.HOME)
    
    def update_settings(self, max_speed: float, max_accel: float, 
                       pid_p: float, pid_i: float, pid_d: float):
        """Update Arduino motor and PID settings"""
        params = [max_speed, max_accel, pid_p, pid_i, pid_d]
        return self.send_command(CommandType.SETTINGS, params)
    
    def get_feedback(self, timeout: float = 0.01) -> Optional[ArduinoFeedback]:
        """Get latest feedback from Arduino"""
        try:
            feedback = self.feedback_queue.get(timeout=timeout)
            return feedback
        except queue.Empty:
            return self.last_feedback
    
    def get_status(self) -> Dict:
        """Get current status"""
        with self.lock:
            return {
                'connected': self.connected,
                'current_position': self.current_position,
                'target_position': self.target_position,
                'is_moving': self.is_moving,
                'command_queue_size': self.command_queue.qsize(),
                'feedback_queue_size': self.feedback_queue.qsize(),
                'latency': self.latency,
                'errors': self.errors,
                'command_count': self.command_count,
                'feedback_count': self.feedback_count
            }
    
    def _send_settings(self):
        """Send initial settings to Arduino"""
        self.update_settings(
            self.config.get('max_speed', 5000),
            self.config.get('max_acceleration', 2000),
            self.config.get('pid_p', 1.0),
            self.config.get('pid_i', 0.0),
            self.config.get('pid_d', 0.1)
        )
    
    def _read_loop(self):
        """Thread loop for reading from Arduino"""
        buffer = ""
        
        while self.running:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line:
                            self._process_feedback(line)
                
                time.sleep(0.001)  # Small delay to prevent CPU spinning
                
            except Exception as e:
                self.errors += 1
                if self.error_callback:
                    self.error_callback(f"Read error: {e}")
    
    def _write_loop(self):
        """Thread loop for writing to Arduino"""
        while self.running:
            try:
                command = self.command_queue.get(timeout=0.1)
                
                if self.serial_port and self.serial_port.is_open:
                    self.serial_port.write(f"{command}\n".encode('utf-8'))
                    self.command_count += 1
                    
            except queue.Empty:
                continue
            except Exception as e:
                self.errors += 1
                if self.error_callback:
                    self.error_callback(f"Write error: {e}")
    
    def _process_feedback(self, line: str):
        """Process feedback line from Arduino"""
        try:
            if line.startswith('FB:'):
                # Parse feedback: FB:current_angle,target_angle,speed,is_moving,timestamp
                parts = line[3:].split(',')
                if len(parts) >= 5:
                    feedback = ArduinoFeedback(
                        current_angle=float(parts[0]),
                        target_angle=float(parts[1]),
                        current_speed=float(parts[2]),
                        is_moving=bool(int(parts[3])),
                        timestamp=int(parts[4]),
                        raw_data=line
                    )
                    
                    # Update state
                    with self.lock:
                        self.current_position = feedback.current_angle
                        self.target_position = feedback.target_angle
                        self.is_moving = feedback.is_moving
                        self.last_feedback = feedback
                        self.last_feedback_time = time.time()
                        self.feedback_count += 1
                        
                        # Calculate latency
                        if self.last_feedback_time > 0:
                            self.latency = (time.time() - self.last_feedback_time) * 1000
                    
                    # Add to queue
                    try:
                        self.feedback_queue.put_nowait(feedback)
                    except queue.Full:
                        # Remove old feedback
                        try:
                            self.feedback_queue.get_nowait()
                            self.feedback_queue.put_nowait(feedback)
                        except queue.Empty:
                            pass
                    
                    # Call callback if set
                    if self.feedback_callback:
                        self.feedback_callback(feedback)
            
            elif line.startswith('MOVE:'):
                # Acknowledgment of move command
                parts = line[5:].split(',')
                if self.feedback_callback:
                    self.feedback_callback(f"Move command acknowledged: {parts[0]} degrees")
            
            elif line.startswith('SETTINGS:'):
                # Settings update acknowledgment
                if self.feedback_callback:
                    self.feedback_callback(f"Settings updated: {line[9:]}")
            
            elif line.startswith('ERROR:'):
                # Error message from Arduino
                self.errors += 1
                if self.error_callback:
                    self.error_callback(f"Arduino error: {line[6:]}")
            
        except Exception as e:
            self.errors += 1
            if self.error_callback:
                self.error_callback(f"Feedback parsing error: {e}")
    
    def set_feedback_callback(self, callback):
        """Set callback for feedback updates"""
        self.feedback_callback = callback
    
    def set_error_callback(self, callback):
        """Set callback for error messages"""
        self.error_callback = callback


class AsyncArduinoScanner:
    """Asynchronously scan for Arduino devices"""
    
    def __init__(self, callback=None):
        self.callback = callback
        self.scanning = False
        self.devices = []
    
    def scan_async(self):
        """Start asynchronous device scan"""
        if self.scanning:
            return
        
        self.scanning = True
        thread = threading.Thread(target=self._scan_worker, daemon=True)
        thread.start()
    
    def _scan_worker(self):
        """Worker thread for device scanning"""
        try:
            self.devices = ArduinoController.list_devices()
            if self.callback:
                self.callback(self.devices)
        finally:
            self.scanning = False
