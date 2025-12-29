# Minimally Working Example

Standalone Python scripts for testing CANopen communication with the RobuROC4 motor controllers, **without** requiring ROS 2.

## Files

| File | Description |
|------|-------------|
| `minimal_example.py` | Main script that connects to the 4 motor controllers via CANopen, initializes them, and sends velocity commands. Demonstrates basic motor control workflow. |
| `gamepad.py` | Xbox 360 controller wrapper using pygame. Provides dead zone handling and easy access to buttons, sticks, triggers, and D-pad. |
| `visualizer.py` | Pygame GUI that displays live gamepad input — useful for testing controller connectivity and calibration. |
| `AMC.eds` | Electronic Data Sheet for the AMC Digiflex motor drives. Defines CANopen object dictionary (registers, PDO mappings, etc.) used by the `canopen` library. |

## Requirements

```bash
pip install canopen pygame
```

- **PCAN USB adapter** connected (or modify `channel` in `minimal_example.py`)
- **Xbox-compatible gamepad** (for `gamepad.py` / `visualizer.py`)

## Usage

**Test motor communication:**
```bash
python minimal_example.py
```

**Test gamepad input:**
```bash
python visualizer.py
```

## Notes

- The scripts use `python-can` with PEAK PCAN interface at 1 Mbps
- Motor nodes are auto-discovered via CANopen scanner
- Velocity commands use CiA 402 profile (SDO 0x60FF)
