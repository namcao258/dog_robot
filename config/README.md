# Configuration Files

Configuration files for Nova-SM3 Robot Dog.

## Structure

```
config/
├── hardware/           # Hardware configurations
│   ├── servo_limits.json
│   ├── servo_home.json
│   └── pinout.json
├── gaits/             # Gait parameters
│   ├── trot.json
│   ├── march.json
│   ├── walk.json
│   └── custom.json
└── sensors/           # Sensor configurations
    ├── mpu6050.json
    ├── ultrasonic.json
    └── pir.json
```

## File Formats

Configuration files use JSON format for easy editing and parsing.

### Hardware Configuration

**servo_limits.json** - Servo min/max positions
```json
{
  "servos": {
    "RFC": {"min": 314, "max": 434},
    "RFF": {"min": 185, "max": 515},
    "RFT": {"min": 365, "max": 607}
  }
}
```

**servo_home.json** - Home positions
```json
{
  "servos": {
    "RFC": 352,
    "RFF": 280,
    "RFT": 510
  }
}
```

### Gait Configuration

**trot.json** - Trot gait parameters
```json
{
  "name": "trot",
  "speed": 5,
  "move_steps": 35,
  "speed_factor": 1.0,
  "step_height_factor": 1.25,
  "diagonal_pairs": [
    ["LF", "RR"],
    ["RF", "LR"]
  ]
}
```

**march.json** - March gait parameters
```json
{
  "name": "march",
  "speed": 5,
  "move_steps": 25,
  "speed_factor": 1.0,
  "step_weight_factor": 1.20,
  "sequence": ["RF", "LR", "LF", "RR"]
}
```

### Sensor Configuration

**mpu6050.json** - IMU settings
```json
{
  "i2c_address": "0x68",
  "update_interval": 40,
  "trigger_threshold": 0.1,
  "oscillation_threshold": 33.0,
  "calibration": {
    "accel_offset_x": 0.0,
    "accel_offset_y": 0.0,
    "gyro_offset_x": 0.0,
    "gyro_offset_y": 0.0
  }
}
```

## Using Configuration Files

### Arduino/C++

Currently, configurations are hardcoded in source files. Future versions will support loading from SD card or EEPROM.

**Current approach:**
```cpp
// In NovaServos.h
float servoHome[TOTAL_SERVOS] = {
  352, 280, 510,  // RF
  // ...
};
```

**Planned approach:**
```cpp
// Load from SD card
loadConfig("config/hardware/servo_home.json");
```

### Python

```python
import json

# Load configuration
with open('config/gaits/trot.json') as f:
    trot_config = json.load(f)

# Use configuration
speed = trot_config['speed']
move_steps = trot_config['move_steps']
```

### ROS2

```python
import yaml

# Load from YAML (alternative format)
with open('config/gaits/trot.yaml') as f:
    trot_params = yaml.safe_load(f)
```

## Creating Custom Configurations

### Custom Gait Example

1. Copy existing gait: `cp gaits/trot.json gaits/my_gait.json`
2. Edit parameters:
```json
{
  "name": "my_custom_gait",
  "speed": 3,
  "move_steps": 40,
  "description": "My custom movement pattern"
}
```
3. Test and adjust parameters
4. Document your gait in comments

### Custom Servo Limits

**Warning:** Incorrect limits can cause servo damage or collisions!

1. Run calibration tool
2. Manually test each servo
3. Record safe min/max values
4. Update configuration
5. Test movements carefully

## Configuration Templates

Templates with default values are provided:
- `hardware/servo_limits.template.json`
- `gaits/trot.template.json`
- `sensors/mpu6050.template.json`

Copy templates and customize for your robot.

## Environment-Specific Configs

For different environments or robots:

```
config/
├── production/       # Main robot configuration
├── development/      # Test/dev configuration
└── simulation/       # Simulator configuration
```

## Validation

**Important:** Validate configurations before use!

Future versions will include:
```bash
# Validate configuration
python tools/validate_config.py config/gaits/trot.json

# Apply configuration
python tools/apply_config.py config/gaits/trot.json
```

## Best Practices

1. **Backup originals** before editing
2. **Document changes** in commit messages
3. **Test incrementally** - small changes first
4. **Keep limits safe** - conservative values
5. **Version control** - track all changes
6. **Use templates** as starting points

## Safety Notes

**Servo Limits:**
- Never exceed physical range of motion
- Leave 5-10 degree safety margin
- Test limits slowly and carefully
- Watch for leg-to-leg collisions

**Speed Settings:**
- Start slow when testing
- Increase gradually
- Monitor servo temperatures
- Listen for grinding/straining

**Gait Parameters:**
- Test new gaits at low speed
- Verify stability before full speed
- Check center of gravity
- Ensure 3-point contact (minimum)

## Troubleshooting

**Invalid configuration:**
- Check JSON syntax
- Verify all required fields
- Check value ranges
- Validate against schema

**Servo behavior incorrect:**
- Verify pin mappings
- Check home positions
- Confirm limit values
- Re-run calibration

## Contributing

See [CONTRIBUTING.md](../CONTRIBUTING.md) for guidelines on submitting configuration improvements.

## Version History

- **v1.0** - Initial configuration structure
- **v1.1** - Planned: JSON loading support
- **v2.0** - Planned: Dynamic configuration updates
