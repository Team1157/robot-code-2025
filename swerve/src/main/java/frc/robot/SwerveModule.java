package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.system.plant.DCMotor;

public class SwerveModule {
  private static final double kWheelRadius = 0.1016; // 4 inch wheel radius in meters
  private static final int kEncoderResolution = 1024; // Lamprey is 10-bit

  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private final TalonFX m_driveMotor; // Falcon 500 (TalonFX) for driving
  private final WPI_TalonSRX m_turningMotor; // TalonSRX for turning

  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  // Set a high velocity limit for turning PID controller to effectively have no speed limit, but retain the acceleration limit
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      1, 
      0.2, 
      0.1, 
      new TrapezoidProfile.Constraints(Double.MAX_VALUE, kModuleMaxAngularAcceleration) // No speed limit, but max accel remains
  );
  
  // Simulation objects
  private final FlywheelSim m_driveMotorSim = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.025);
  private final FlywheelSim m_turningMotorSim = new FlywheelSim(DCMotor.getVex775Pro(1), 1.0, 0.01);

  // Simulation variables for tracking position
  private double driveMotorSimPosition = 0;
  private double turningMotorSimPosition = 0;

  // NetworkTables
  private final NetworkTableInstance ntInstance;
  private final NetworkTable ntTable;

  /**
   * Constructs a SwerveModule with a drive motor (TalonFX) and turning motor
   * (TalonSRX).
   *
   * @param driveMotorChannel   CAN ID for the TalonFX drive motor.
   * @param turningMotorChannel CAN ID for the TalonSRX turning motor.
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel) {
    m_driveMotor = new TalonFX(driveMotorChannel); // TalonFX for driving
    m_turningMotor = new WPI_TalonSRX(turningMotorChannel); // TalonSRX for turning

    // Configure the Lamprey2 encoder in quadrature mode
    m_turningMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10); // Feedback device is set to Quadrature Encoder

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
  private long lastPrintTime = 0;

  public void setDesiredState(SwerveModuleState desiredState) {
      var encoderRotation = new Rotation2d(m_turningMotor.getSelectedSensorPosition() * 2 * Math.PI / kEncoderResolution);
  
      // Optimize the reference state to avoid spinning the thingy further than 90 degrees
      SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);
  
      // Calculate the drive output from the drive PID controller
      final double driveOutput = m_drivePIDController.calculate(
          m_driveMotor.getVelocity().getValue() * kWheelRadius * 2 * Math.PI / kEncoderResolution,
          state.speedMetersPerSecond);
  
      // Calculate the turning motor output from the turning PID controller
      final double turnOutput = m_turningPIDController.calculate(
          m_turningMotor.getSelectedSensorPosition() * 2 * Math.PI / kEncoderResolution,
          state.angle.getRadians());
  
      // Publish desired state to NetworkTables
      ntTable.getEntry("DesiredSpeed").setDouble(state.speedMetersPerSecond);
      ntTable.getEntry("DesiredAngle").setDouble(state.angle.getDegrees());
  
      // Set motor outputs
      m_driveMotor.set(driveOutput);
      m_turningMotor.set(ControlMode.Position, turnOutput);
  
      // Print turnOutput every second so i can debuggggggg
      long currentTime = System.currentTimeMillis();
      if (currentTime - lastPrintTime >= 4000) {
          System.out.println("Turn Output: " + turnOutput);
          lastPrintTime = currentTime;
      }
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
      System.out.println("SimDrivePosition" + driveMotorSimPosition);
      System.out.println("SimDriveVelocity" + driveVelocityRadPerSec);
      
      System.out.println("SimTurningPosition"+ turningMotorSimPosition);
      System.out.println("SimTurningVelocity"+ turningVelocityRadPerSec);
  }
}
