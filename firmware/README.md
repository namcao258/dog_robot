# Firmware

Arduino firmware for Nova-SM3 Robot Dog.

## Structure

```
firmware/
├── src/                    # Main source code
│   ├── main/              # Main firmware
│   ├── slave/             # Slave Arduino code
│   └── bootloader/        # Custom bootloader (optional)
├── include/               # Header files
│   ├── config.h          # Configuration constants
│   ├── servos.h          # Servo definitions
│   └── sensors.h         # Sensor interfaces
├── lib/                   # Custom libraries
│   ├── AsyncServo/       # Non-blocking servo control
│   ├── GaitEngine/       # Gait algorithms
│   └── SensorFusion/     # Multi-sensor integration
├── examples/              # Example sketches
│   ├── basic_movement/   # Simple movement test
│   ├── sensor_test/      # Sensor testing
│   └── calibration/      # Calibration tools
└── tests/                 # Hardware tests
    ├── servo_test/       # Test individual servos
    ├── sensor_test/      # Test sensors
    └── integration/      # Full system tests
```

## Building

### Requirements
- Arduino IDE 1.8.13 or later
- Required libraries (see lib/README.md)

### Compilation
1. Open `src/main/Nova-SM3_mega-v4.2.ino` in Arduino IDE
2. Select board: Arduino Mega 2560
3. Select port
4. Click Upload

### Configuration
Edit `include/config.h` to customize:
- Enable/disable modules
- Adjust servo limits
- Set sensor parameters
- Configure debug output

## Libraries

### Required External Libraries
Install via Arduino Library Manager:

- **Adafruit PWM Servo Driver Library** (v2.4.0+)
  - For PCA9685 servo controller

- **Adafruit GFX Library** (v1.10.0+)
  - Graphics library for OLED

- **Adafruit SSD1331** (v1.0.0+)
  - OLED display driver

- **Adafruit NeoPixel** (v1.8.0+)
  - RGB LED control

- **PS2X Library** (v1.6+)
  - PS2 controller support

- **Wire** (built-in)
  - I2C communication

- **SPI** (built-in)
  - SPI communication

- **EEPROM** (built-in)
  - Save calibration data

### Custom Libraries
Located in `lib/`:

- **AsyncServo** - Non-blocking servo movement
- **NovaServos** - Servo configuration and data

## Memory Usage

Current usage (Arduino Mega):
- **Sketch:** 36% of program storage (58,464 bytes)
- **RAM:** 33% of dynamic memory (2,688 bytes)

**Tips to reduce memory:**
- Disable unused modules in config.h
- Remove debug output (set debug = 0)
- Optimize string constants with F() macro

## Pinout

### Arduino Mega

**I2C (Communication):**
- SDA: Pin 20
- SCL: Pin 21

**PWM Controller:**
- OE (Output Enable): Pin 3

**PS2 Controller:**
- DAT: Pin 24
- CMD: Pin 25
- SEL: Pin 26
- CLK: Pin 27

**Sensors:**
- PIR Front: Pin 14
- PIR Left: Pin 15
- PIR Right: Pin 16

**Other:**
- Buzzer: Pin 11
- Power Control: Pin 4
- Battery Monitor: A1
- Current Monitor: A2

### TCA9548 I2C Multiplexer Channels
- Channel 0: Null/Reserved
- Channel 1: PWM Controller
- Channel 2: Slave Arduino
- Channel 3: MPU6050

## Configuration

### Enable/Disable Modules
Edit these flags in the main .ino file:

```cpp
byte slave_active = 1;    // Slave Arduino
byte pwm_active = 1;      // PWM controller
byte ps2_active = 0;      // PS2 controller
byte serial_active = 1;   // Serial commands
byte mpu_active = 0;      // MPU6050
byte rgb_active = 1;      // RGB LEDs
byte oled_active = 1;     // OLED display
byte pir_active = 0;      // PIR sensors
byte uss_active = 0;      // Ultrasonic sensors
byte amp_active = 0;      // Current monitoring
byte batt_active = 0;     // Battery monitoring
byte buzz_active = 1;     // Buzzer sounds
byte melody_active = 0;   // Melodic sounds
```

### Speed Settings
```cpp
const float min_spd = 96.0;     // Slowest speed
const float max_spd = 1.0;      // Fastest speed
float default_spd = 13.0;       // Default speed
```

### Debug Output
```cpp
const byte debug = 1;     // General messages
const byte debug1 = 1;    // PS2 & PIR
const byte debug2 = 0;    // Servo steps
const byte debug3 = 0;    // Ramping
const byte debug4 = 0;    // Power monitoring
const byte debug5 = 1;    // MPU & USS
const byte debug6 = 0;    // Serial communication
```

## Serial Commands

Default baud rate: **19200**

See `docs/api/serial_protocol.md` for complete command reference.

### Basic Commands
- `s` - Stand/Stay
- `i` - Sit
- `t` - Trot
- `m` - March
- `f` - Forward
- `b` - Backward

## Calibration

**Important:** Calibrate servos before first use!

1. Upload `examples/calibration/Nova-SM3-calibrate.ino`
2. Follow calibration procedure in `docs/calibration/servo_calibration.md`
3. Update servo home positions in `NovaServos.h`
4. Update servo limits in `NovaServos.h`
5. Re-upload main firmware

## Testing

### Servo Test
```bash
# Upload tests/servo_test/servo_test.ino
# Each servo moves individually
```

### Sensor Test
```bash
# Upload tests/sensor_test/sensor_test.ino
# Displays sensor readings via serial
```

### Integration Test
```bash
# Upload tests/integration/full_system_test.ino
# Tests all systems together
```

## Troubleshooting

**Compilation Errors:**
- Install missing libraries
- Check Arduino IDE version
- Verify board selection

**Upload Errors:**
- Check USB connection
- Select correct port
- Try different USB cable
- Reset Arduino before upload

**Runtime Errors:**
- Check serial monitor for debug messages
- Verify power supply (7.4V-11.1V)
- Check servo connections
- Recalibrate servos

## Performance Optimization

### Speed Optimization
- Reduce serial output
- Minimize delay() calls
- Use non-blocking code
- Optimize loop frequency

### Memory Optimization
- Use PROGMEM for constants
- Use F() macro for strings
- Minimize global variables
- Free unused resources

## Version History

- **v4.2** - Added TCA9548 multiplexer
- **v4.1** - Initial stable release

## Contributing

See main [CONTRIBUTING.md](../CONTRIBUTING.md)

## License

See main [LICENSE](../LICENSE)
