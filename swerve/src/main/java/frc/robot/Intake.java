package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.XboxController;

public class Intake {

  private static final double kIntakeSpeed = 100; //percent voltage we run intake at
  private final WPI_TalonSRX intakeMotor;
  private final XboxController controller;

  public Intake(int motorPort, XboxController controller) {
    intakeMotor = new WPI_TalonSRX(motorPort);
    this.controller = controller;
    
    // cause we don't need to worry about pid or anything
    intakeMotor.configFactoryDefault();
  }

  public void runIntake() {
    // Set the motor to the speed
    intakeMotor.set(ControlMode.PercentOutput, kIntakeSpeed);
  }

  public void stop() {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public void teleopPeriodic() {
    // i think this is the a button? i neeeeed to test 
    if (controller.getRawButton(3)) {
      runIntake();
    } else {
      stop();
    }
  }
}
