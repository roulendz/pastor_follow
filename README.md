# Human Tracking System with Servo Control

A real-time human tracking system using computer vision and servo motor control. The system uses MediaPipe for pose detection, OpenCV for video processing, and Arduino for precise stepper motor control with PID feedback.

## Features

- **Real-time Human Pose Detection**: Uses MediaPipe for accurate body pose tracking
- **Smooth Motor Control**: PID controller with adaptive tuning for smooth movement
- **Bidirectional Communication**: Arduino provides real-time feedback on motor position
- **Non-blocking GUI**: Asynchronous device scanning and threading prevents UI freezing
- **Modular Architecture**: Clean separation of concerns for easy maintenance
- **Multiple Tracking Modes**: Center, largest person, or closest person tracking
- **Performance Monitoring**: Real-time graphs showing error, PID output, and motor position
- **Configurable Settings**: Save and load configurations in JSON format

## Hardware Requirements

1. **Windows PC** with USB ports
2. **Video Capture Device** (webcam or capture card)
3. **Arduino UNO**
4. **DM542 Stepper Motor Driver**
5. **Stepper Motor** with 180:1 gear ratio
6. **Power Supply** for stepper motor (12-48V DC)
7. **Jumper wires** for connections

## Hardware Connections

### DM542 to Arduino Connections (Common Cathode):

| Arduino Pin | DM542 Terminal | Description |
|-------------|----------------|-------------|
| D2 | DIR+ | Direction control signal |
| D3 | PUL+ | Step pulse signal (PWM capable) |
| D4 | ENA+ | Enable signal |
| GND | DIR- | Direction ground |
| GND | PUL- | Step ground |
| GND | ENA- | Enable ground |

**Note:** This uses common cathode configuration where all negative terminals (DIR-, PUL-, ENA-) connect to Arduino GND, and the Arduino directly drives the positive terminals.

### DM542 to Motor/Power:

| Terminal | Connection |
|----------|------------|
| A+/A- | Motor coil A |
| B+/B- | Motor coil B |
| DC+/DC- | Power supply (12-48V) |

### DM542 DIP Switch Settings:
- Set microstepping to 8 (1600 steps/rev)
- Set current according to your motor specifications
- Enable motor driver

## Software Installation

### Prerequisites
- Python 3.8 or higher
- Arduino IDE (for uploading code to Arduino)

### Step 1: Clone or Download the Project
```bash
# Create a project folder
mkdir human_tracking_system
cd human_tracking_system

# Copy all provided files to this folder
```

### Step 2: Install Python Dependencies

#### Windows (using provided script):
```batch
install.bat
```

#### Manual installation:
```bash
# Create virtual environment
python -m venv venv

# Activate virtual environment
# Windows:
venv\Scripts\activate
# Linux/Mac:
source venv/bin/activate

# Install requirements
pip install -r requirements.txt
```

### Step 3: Upload Arduino Code
1. Open Arduino IDE
2. Install the **AccelStepper** library:
   - Go to Tools → Manage Libraries
   - Search for "AccelStepper"
   - Install the library by Mike McCauley
3. Open `arduino_stepper_controller.ino`
4. Select your Arduino UNO board and port
5. Upload the code

### Step 4: Configure the System
1. Edit `config.json` (created on first run) to match your setup:
   - Set correct COM port for Arduino
   - Adjust PID gains if needed
   - Configure video resolution

## Running the Application

1. **Connect Hardware**:
   - Connect Arduino to PC via USB
   - Connect capture card/webcam
   - Power on the stepper motor driver

2. **Start the Application**:
```bash
# Activate virtual environment (if not already active)
venv\Scripts\activate  # Windows
# or
source venv/bin/activate  # Linux/Mac

# Run the application
python main_app.py
```

3. **Initial Setup in GUI**:
   - Click "Scan" next to Video to find capture devices
   - Select your video device
   - Click "Scan" next to Arduino to find COM ports
   - Select Arduino COM port
   - Click "Connect" to establish Arduino connection

4. **Start Tracking**:
   - Position yourself in front of the camera
   - Click "Start Tracking" to begin
   - The motor will automatically track your position

## GUI Controls

### Main Controls
- **Start/Stop Tracking**: Enable/disable automatic tracking
- **Home**: Return motor to center position (0°)
- **Emergency Stop**: Immediately stop all motor movement
- **Record**: Record video feed (saves to file)

### PID Control Tab
- **P Gain**: Proportional gain (responsiveness)
- **I Gain**: Integral gain (steady-state error correction)
- **D Gain**: Derivative gain (overshoot dampening)
- **Max Speed/Acceleration**: Motor speed limits
- **Adaptive PID**: Enable automatic gain tuning

### Tracking Tab
- **Tracking Mode**: 
  - Center: Track person closest to center
  - Largest: Track largest person
  - Closest: Track person with highest confidence
- **Target Body Part**: Which body part to track (nose, shoulders, etc.)
- **Smoothing**: Enable position smoothing to reduce jitter
- **Min Confidence**: Minimum detection confidence threshold

### Monitor Tab
- Real-time graphs showing:
  - Tracking error
  - PID controller output
  - Motor position (actual vs target)
- Performance statistics

### Logs Tab
- System logs and debug information
- Save logs to file for debugging

## Troubleshooting

### Arduino Not Connecting
- Check COM port selection
- Verify Arduino has the code uploaded
- Check USB cable connection
- Try different USB port
- Check Arduino Serial Monitor isn't open

### Motor Not Moving
- Verify power supply is connected and on
- Check all wiring connections
- Verify DM542 DIP switches are configured correctly
- Check enable signal (if using)
- Test with manual commands in Arduino Serial Monitor

### Video Not Working
- Check capture device is connected
- Try different video backend (DirectShow vs MSMF)
- Verify capture device works in other applications
- Check Windows privacy settings for camera access

### Tracking Issues
- Ensure good lighting conditions
- Stand 2-3 meters from camera
- Wear contrasting clothing
- Adjust confidence threshold
- Try different tracking modes

### Performance Issues
- Reduce video resolution in config.json
- Disable skeleton display
- Reduce MediaPipe model complexity
- Close other applications

## PID Tuning Guide

1. **Start with P only** (I=0, D=0):
   - Increase P until system responds quickly
   - If oscillating, reduce P

2. **Add D to reduce overshoot**:
   - Increase D gradually
   - Too much D causes jittery movement

3. **Add I to eliminate steady-state error**:
   - Small amounts of I
   - Too much I causes oscillation

4. **Use Adaptive PID** for automatic tuning

## System Architecture

```
┌─────────────────────┐
│   Video Capture     │
│  (capture card)     │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│   Pose Detection    │
│   (MediaPipe)       │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│   PID Controller    │
│  (position->angle)  │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  Arduino Controller │
│  (serial comm)      │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│    Arduino UNO      │
│  (stepper control)  │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│   DM542 Driver      │
│  (motor driver)     │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│   Stepper Motor     │
│   (180:1 ratio)     │
└─────────────────────┘
```

## Configuration File (config.json)

The system automatically creates a configuration file with these sections:
- `video`: Capture settings
- `pose_detection`: MediaPipe settings
- `pid`: Controller gains and limits
- `arduino`: Serial communication settings
- `tracking`: Tracking behavior
- `gui`: Interface preferences
- `performance`: Optimization settings

## Development

### Adding New Features
The modular design makes it easy to extend:
- `video_capture.py`: Video input handling
- `pose_detection.py`: Computer vision processing
- `pid_controller.py`: Control algorithms
- `arduino_controller.py`: Hardware communication
- `main_app.py`: GUI and integration

### Testing
Test individual modules:
```python
# Test video capture
python -c "from video_capture import VideoCapture; print(VideoCapture.list_devices())"

# Test Arduino connection
python -c "from arduino_controller import ArduinoController; print(ArduinoController.list_devices())"
```

## Safety Notes

⚠️ **WARNING**: 
- Always have emergency stop ready
- Keep hands clear of moving parts
- Use appropriate current limiting on motor driver
- Ensure proper grounding of all components
- Never exceed motor voltage ratings

## License

This project is provided as-is for educational purposes.

## Support

For issues:
1. Check the Logs tab for error messages
2. Verify all connections
3. Test components individually
4. Review configuration settings
5. Check the troubleshooting section

## Credits

- MediaPipe by Google
- OpenCV community
- AccelStepper library by Mike McCauley
- CustomTkinter by Tom Schimansky
