package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SwerveModule {
    private static final double kWheelRadius = 0.1016; // 4 inch wheel radius in meters
    private static final int kEncoderResolution = 1024; // lamprey is 10 bit

    private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    private final TalonFX m_driveMotor; // Falcon 500 (TalonFX) for driving
    private final TalonSRX m_turningMotor; // TalonSRX for turning

    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);
    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
        1,
        0,
        0,
        new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration)
    );

    // NetworkTables
    private final NetworkTableInstance ntInstance;
    private final NetworkTable ntTable;

    /**
     * Constructs a SwerveModule with a drive motor (TalonFX) and turning motor (TalonSRX).
     *
     * @param driveMotorChannel CAN ID for the TalonFX drive motor.
     * @param turningMotorChannel CAN ID for the TalonSRX turning motor.
     */
    public SwerveModule(int driveMotorChannel, int turningMotorChannel) {
        m_driveMotor = new TalonFX(driveMotorChannel); // TalonFX for driving
        m_turningMotor = new TalonSRX(turningMotorChannel); // TalonSRX for turning

        // Initialize NetworkTables
        ntInstance = NetworkTableInstance.getDefault();
        ntTable = ntInstance.getTable("SwerveModule" + driveMotorChannel);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        double driveVelocity = m_driveMotor.getVelocity().getValue() * kWheelRadius * 2 * Math.PI / kEncoderResolution;
        double turningPosition = m_turningMotor.getSelectedSensorPosition() * 2 * Math.PI / kEncoderResolution;

        // Publish data to NetworkTables
        ntTable.getEntry("DriveVelocity").setDouble(driveVelocity);
        ntTable.getEntry("TurningPosition").setDouble(turningPosition);

        return new SwerveModuleState(driveVelocity, new Rotation2d(turningPosition));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        double driveDistance = m_driveMotor.getPosition().getValue() * kWheelRadius * 2 * Math.PI / kEncoderResolution;
        double turningPosition = m_turningMotor.getSelectedSensorPosition() * 2 * Math.PI / kEncoderResolution;

        // Publish data to NetworkTables
        ntTable.getEntry("DriveDistance").setDouble(driveDistance);
        ntTable.getEntry("TurningPosition").setDouble(turningPosition);

        return new SwerveModulePosition(driveDistance, new Rotation2d(turningPosition));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        var encoderRotation = new Rotation2d(m_turningMotor.getSelectedSensorPosition() * 2 * Math.PI / kEncoderResolution);

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = m_drivePIDController.calculate(
            m_driveMotor.getVelocity().getValue() * kWheelRadius * 2 * Math.PI / kEncoderResolution, 
            state.speedMetersPerSecond
        );

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = m_turningPIDController.calculate(
            m_turningMotor.getSelectedSensorPosition() * 2 * Math.PI / kEncoderResolution, 
            state.angle.getRadians()
        );

        // Publish desired state to NetworkTables
        ntTable.getEntry("DesiredSpeed").setDouble(state.speedMetersPerSecond);
        ntTable.getEntry("DesiredAngle").setDouble(state.angle.getDegrees());

        // Set motor outputs
        m_driveMotor.set(driveOutput);
        m_turningMotor.set(ControlMode.Position, turnOutput);
    }
}
