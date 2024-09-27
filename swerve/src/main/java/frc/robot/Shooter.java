package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.XboxController;

public class Shooter {

  private static final double kShooterSpeed = 100; //percent voltage we shoot at
  private final WPI_TalonSRX shooterMotor;
  private final XboxController controller;

  public Shooter(int motorPort, XboxController controller) {
    shooterMotor = new WPI_TalonSRX(motorPort);
    this.controller = controller;
    
    // cause we don't need to worry about pid or anything
    shooterMotor.configFactoryDefault();
  }

  public void shoot() {
    // Set the motor to the speed
    shooterMotor.set(ControlMode.PercentOutput, kShooterSpeed);
  }

  public void stop() {
    shooterMotor.set(ControlMode.PercentOutput, 0);
  }

  public void teleopPeriodic() {
    // x button on the gc con shoots
    if (controller.getRawButton(1 )) {
      shoot();
    } else {
      stop();
    }
  }
}
