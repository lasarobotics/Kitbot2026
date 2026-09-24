package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends SubsystemBase {

  private static DriveSubsystem s_driveSubsystem;
  TalonFX m_leftFrontDriveMotor;
  TalonFX m_rightFrontDriveMotor;
  TalonFX m_leftBackDriveMotor;
  TalonFX m_rightBackDriveMotor;

  DifferentialDrive m_robotDrive;

  DoubleSupplier m_driveRequest;
  DoubleSupplier m_turnRequest;

  public static DriveSubsystem getInstance() {
    if (s_driveSubsystem == null) {
      s_driveSubsystem = new DriveSubsystem();
    }
    return s_driveSubsystem;
  }

  public void configureBindings(DoubleSupplier driveRequest, DoubleSupplier turnRequest) {
    m_driveRequest = driveRequest;
    m_turnRequest = turnRequest;
  }

  public DriveSubsystem() {
    m_leftFrontDriveMotor = new TalonFX(10);
    m_rightFrontDriveMotor = new TalonFX(11);
    m_leftBackDriveMotor = new TalonFX(12);
    m_rightBackDriveMotor = new TalonFX(13);

    m_leftBackDriveMotor.setControl(new Follower(10, MotorAlignmentValue.Aligned));
    m_rightBackDriveMotor.setControl(new Follower(11, MotorAlignmentValue.Aligned));

    m_robotDrive =
        new DifferentialDrive(
            (double speed) -> {
              m_leftFrontDriveMotor.set(speed);
            },
            (double speed) -> {
              m_rightFrontDriveMotor.set(-speed);
            });
  }

  @Override
  public void periodic() {
    double driveRequest = m_driveRequest.getAsDouble();
    double turnRequest = m_turnRequest.getAsDouble();

    m_robotDrive.arcadeDrive(driveRequest, turnRequest, true);

    Logger.recordOutput("DriveSubsystem/driveRequest", m_driveRequest.getAsDouble());
    Logger.recordOutput("DriveSubsystem/turnRequest", m_turnRequest.getAsDouble());
    DifferentialDrive.WheelSpeeds wheelSpeeds =
        DifferentialDrive.arcadeDriveIK(driveRequest, turnRequest, true);
    Logger.recordOutput("DriveSubsystem/leftWheelWantedDutyCycle", wheelSpeeds.left);
    Logger.recordOutput("DriveSubsystem/rightWheelWantedDutyCycle", wheelSpeeds.right);
    Logger.recordOutput(
        "DriveSubsystem/leftWheelActualDutyCycle", m_leftFrontDriveMotor.getDutyCycle().getValue());
    Logger.recordOutput(
        "DriveSubsystem/rightWheelActualDutyCycle",
        -m_rightFrontDriveMotor.getDutyCycle().getValue());
  }
}
