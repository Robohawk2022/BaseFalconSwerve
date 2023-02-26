# Team 3373 2023 competition bot

# TODO

# Prior to delivery

- Test arm code
  - Calibration routine to set min/max
  - Move to preset
  - Teleop

- Develop autonomous programs

- Shuffleboard configuration & stats

# Post delivery

- General wiring
  - Connect to all motors and
    - Update firmware
    - Validate CAN bus ID
    - Validate position (inverted or not)

- Arm subsystem
  - Measurements
    - Arm extension (rate & travel range)
    - Arm rotation (rate & travel range)
  - Test
    - Teleop (use this to establish presets - x8)
    - Presets (tune these to be as fast/accurate as possible)
  
- Hand subsystem
  - Test grab/release (tune timing of bumper)

- Vision subsystem
  - Test aligning to tag (tune distance from tag and speeds)

- Balance subsystem
  - Test and tune speed profile
