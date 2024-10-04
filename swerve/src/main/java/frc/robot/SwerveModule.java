package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.MathUtil;

public class SwerveModule {
    private static final double kWheelRadius = 0.1016; // 4 inch wheel radius in meters
    private static final int kEncoderResolution = 1024; // Lamprey is 10-bit
    private static final double kMaxVoltage = 12.0; // Maximum voltage for the turning motor

    private final TalonFX m_driveMotor; // Falcon 500 (TalonFX) for driving
    private final WPI_TalonSRX m_turningMotor; // TalonSRX for turning

    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

    // Simulation objects
    private final FlywheelSim m_driveMotorSim = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.025);
    private final FlywheelSim m_turningMotorSim = new FlywheelSim(DCMotor.getVex775Pro(1), 1.0, 0.01);

    // Simulation variables for tracking position
    private double driveMotorSimPosition = 0;
    private double turningMotorSimPosition = 0;

    // NetworkTables
    private final NetworkTableInstance ntInstance;
    private final NetworkTable ntTable;

    // PID constants for turning control
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private final PIDController m_turningPIDController = new PIDController(kP, kI, kD);

    /**
     * Constructs a SwerveModule with a drive motor (TalonFX) and turning motor (TalonSRX).
     *
     * @param driveMotorChannel   CAN ID for the TalonFX drive motor.
     * @param turningMotorChannel CAN ID for the TalonSRX turning motor.
     */
    public SwerveModule(int driveMotorChannel, int turningMotorChannel) {
        m_driveMotor = new TalonFX(driveMotorChannel);
        m_turningMotor = new WPI_TalonSRX(turningMotorChannel);
        m_turningMotor.setNeutralMode(NeutralMode.Brake); // Set the turning motor to brake mode

        // Configure the Lamprey2 encoder in quadrature mode
        m_turningMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

        // Initialize NetworkTables
        ntInstance = NetworkTableInstance.getDefault();
        ntTable = ntInstance.getTable("SwerveModule" + driveMotorChannel);

        // Configure the turning PID controller
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        double driveVelocity = m_driveMotor.getVelocity().getValue() * kWheelRadius * 2 * Math.PI / kEncoderResolution;
        double turningPosition = getTurningPosition();

        ntTable.getEntry("TurningEncoderTicks").setDouble(m_turningMotor.getSelectedSensorPosition());
        ntTable.getEntry("TurningPosition").setDouble(turningPosition);
        ntTable.getEntry("DriveVelocity").setDouble(driveVelocity);

        return new SwerveModuleState(driveVelocity, new Rotation2d(turningPosition));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        double driveDistance = m_driveMotor.getPosition().getValue() * kWheelRadius * 2 * Math.PI / kEncoderResolution;
        double turningPosition = getTurningPosition();

        ntTable.getEntry("TurningEncoderTicks").setDouble(m_turningMotor.getSelectedSensorPosition());
        ntTable.getEntry("TurningPosition").setDouble(turningPosition);
        ntTable.getEntry("DriveDistance").setDouble(driveDistance);

        return new SwerveModulePosition(driveDistance, new Rotation2d(turningPosition));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        var currentRotation = new Rotation2d(getTurningPosition());
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);

        // Calculate the drive output from the drive PID controller
        final double driveOutput = m_drivePIDController.calculate(
            m_driveMotor.getVelocity().getValue() * kWheelRadius * 2 * Math.PI / kEncoderResolution,
            state.speedMetersPerSecond);

        // Calculate the turning motor output
        double turningOutput = calculateTurningMotorOutput(state.angle.getRadians());

        // Set motor outputs
        m_driveMotor.set(driveOutput);
        m_turningMotor.setVoltage(turningOutput*100);

        // Publish desired state to NetworkTables
        ntTable.getEntry("DesiredSpeed").setDouble(state.speedMetersPerSecond);
        ntTable.getEntry("DesiredAngle").setDouble(state.angle.getDegrees());
        ntTable.getEntry("TurningOutput").setDouble(turningOutput);
    }

    /**
     * Calculates the turning motor output based on the current position and desired angle.
     *
     * @param targetAngleRadians The target angle in radians.
     * @return The voltage to apply to the turning motor.
     */
    private double calculateTurningMotorOutput(double targetAngleRadians) {
        double currentAngle = getTurningPosition();
        double output = m_turningPIDController.calculate(currentAngle, targetAngleRadians);
        return MathUtil.clamp(output, -kMaxVoltage, kMaxVoltage);
    }

    /**
     * Gets the current turning position in radians.
     *
     * @return The current turning position in radians.
     */
    private double getTurningPosition() {
        return m_turningMotor.getSelectedSensorPosition() * 2 * Math.PI / kEncoderResolution;
    }

    /**
     * Updates the simulation for the drive and turning motors.
     *
     * This method should be called periodically during simulation.
     */
    public void simulationPeriodic(double dt) {
        // Update flywheel simulations
        m_driveMotorSim.update(dt);
        m_turningMotorSim.update(dt);

        // Simulate position by integrating angular velocity
        double driveVelocityRadPerSec = m_driveMotorSim.getAngularVelocityRadPerSec();
        double turningVelocityRadPerSec = m_turningMotorSim.getAngularVelocityRadPerSec();

        driveMotorSimPosition += driveVelocityRadPerSec * dt;
        turningMotorSimPosition += turningVelocityRadPerSec * dt;

        // Publish simulation data to NetworkTables
        ntTable.getEntry("SimDrivePosition").setDouble(driveMotorSimPosition);
        ntTable.getEntry("SimDriveVelocity").setDouble(driveVelocityRadPerSec);
        ntTable.getEntry("SimTurningPosition").setDouble(turningMotorSimPosition);
        ntTable.getEntry("SimTurningVelocity").setDouble(turningVelocityRadPerSec);
    }
}