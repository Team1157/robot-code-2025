# Button Mapping:

- Swerve translation: left stick
- Swerve rotation: right stick
- Gyro reset: right bumper bottomed out
- Callibrate gyro: A button
- Intake motor: need to assign
- Shooter motor: need to assign

# CAN IDs:

1. **CAN ID 9** – TalonSRX for the shooter.
   - Connected to the shooter motor, controlled by the `Shooter` class.
1. **CAN ID 10** – TalonSRX for the intake.
   - Connected to the intake motor, controlled by the `Intake` class.

3. **Drivetrain Motors**:
   - **Drive Motors (TalonFX)**:
     - Front Left Module: CAN ID 1 (drive motor).
     - Front Right Module: CAN ID 3 (drive motor).
     - Rear Left Module: CAN ID 5 (drive motor).
     - Rear Right Module: CAN ID 7 (drive motor).
   - **Steering Motors (TalonSRX)**:
     - Front Left Module: CAN ID 2 (steer motor).
     - Front Right Module: CAN ID 4 (steer motor).
     - Rear Left Module: CAN ID 6 (steer motor).
     - Rear Right Module: CAN ID 8 (steer motor).
