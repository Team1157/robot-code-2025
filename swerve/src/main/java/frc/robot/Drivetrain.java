package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//deprecated but the easoest way for me to make music
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Drivetrain {
    public static final double kMaxSpeed = 6.0; // 6 meters per second
    public static final double kMaxAngularSpeed = 2 * Math.PI; // 1 rotation per second

    private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    private final SwerveModule m_frontLeft = new SwerveModule(1, 2);
    private final SwerveModule m_frontRight = new SwerveModule(3, 4);
    private final SwerveModule m_backLeft = new SwerveModule(5, 6);
    private final SwerveModule m_backRight = new SwerveModule(7, 8);

    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
    public void resetGyro() {
      m_gyro.reset();
    }
    public void calibrateGyro() {
      m_gyro.calibrate();
    }
    // Orchestra for the music lol
    private Orchestra orchestra;

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveDriveOdometry m_odometry;

    private final NetworkTableEntry m_controlModeEntry;
    private final NetworkTableEntry m_gyroOutputEntry;

    private final Field2d m_field2d = new Field2d();

    @SuppressWarnings("removal") //I'm a horrible person
    public Drivetrain() {
      
        m_odometry = new SwerveDriveOdometry(
                m_kinematics,
                m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_backLeft.getPosition(),
                        m_backRight.getPosition()
                });

        NetworkTable table = NetworkTableInstance.getDefault().getTable("Drivetrain");
        m_controlModeEntry = table.getEntry("ControlMode");
        m_gyroOutputEntry = table.getEntry("GyroRotation");

        // Publish control mode options
        publishControlModeOptions();

        // Publish the Field2d object to SmartDashboard
        SmartDashboard.putData("Field", m_field2d);

        orchestra = new Orchestra();
        orchestra.addInstrument(new TalonFX(1));
        orchestra.addInstrument(new TalonFX(3));
        orchestra.addInstrument(new TalonFX(5));
        orchestra.addInstrument(new TalonFX(7));
    }

    public void drive(double xSpeed, double ySpeed, double rot, double periodSeconds) {
        // Field-relative control
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rot, m_gyro.getRotation2d());

        var swerveModuleStates = m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.discretize(chassisSpeeds, periodSeconds));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);

        // Output gyro rotation to NetworkTables for diagnostics
        m_gyroOutputEntry.setDouble(m_gyro.getAngle());
    }

    public void playMusic() {
        // Load and play the song
        orchestra.loadMusic("Wii-Song.chrp");
        orchestra.play();
    }

    public void updateOdometry() {
        m_odometry.update(
                m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_backLeft.getPosition(),
                        m_backRight.getPosition()
                });

        // Update the Field2d object with the current pose
        m_field2d.setRobotPose(getPose());
    }

    private void publishControlModeOptions() {
        String[] options = { "Field-Relative" };
        m_controlModeEntry.setStringArray(options);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
                m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_backLeft.getPosition(),
                        m_backRight.getPosition()
                },
                pose);
    }
}
