package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends LoggedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_swerve = new Drivetrain();
  // Call the shooter.java file that just runs one talon srx for now but if we have a multi motor shooter i'll add more
  private final Shooter m_shooter = new Shooter(9, m_controller); // shooter motorcontroller will be can id 9
  // Call the intake.java file that just runs one talon srx for now but if we have a multi motor shooter i'll add more
  private final Intake m_intake = new Intake(10, m_controller); // shooter motorcontroller will be can id 9
  
  // Slew rate limiters to make joystick inputs more gentle; 1/4 sec from 0 to 1 for drive and 1/3 sec for turning.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(4);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(4);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  private final Drivetrain m_drivetrain = new Drivetrain();

  @Override
  public void robotInit() {
  }

  @Override
  public void simulationPeriodic() {
  }

  @Override
  public void autonomousPeriodic() {
    m_drivetrain.playMusic();
  }

  @Override
  public void teleopPeriodic() {
    // call the shooter to actually work
    m_shooter.teleopPeriodic();
    m_intake.teleopPeriodic();
    driveWithJoystick(true);
    // Reset gyro with z button on the gcc and calibrate gyro with the a button
    if (m_controller.getRawButton(8)) {
      m_drivetrain.resetGyro();
    }    
    if (m_controller.getRawButton(2)) {
      m_drivetrain.calibrateGyro();
    }
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get raw inputs
    double leftX = MathUtil.applyDeadband(m_controller.getLeftX(), 0.1);
    double leftY = MathUtil.applyDeadband(m_controller.getLeftY(), 0.1);

    // Square the inputs while preserving sign
    leftX = Math.copySign(leftX * leftX, leftX);
    leftY = Math.copySign(leftY * leftY, leftY);

    // Apply slew rate limiters and scale inputs
    final var xSpeed = m_xspeedLimiter.calculate(1.2 * leftY) * Drivetrain.kMaxSpeed;
    final var ySpeed = m_yspeedLimiter.calculate(1.2 * leftX) * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation 
    final var rot = m_rotLimiter.calculate(1.2 * MathUtil.applyDeadband(m_controller.getRawAxis(5), 0.1))
        * Drivetrain.kMaxAngularSpeed;

    // Drive the swerve with square inputs
    m_swerve.drive(xSpeed, ySpeed, rot, getPeriod());
}

}
