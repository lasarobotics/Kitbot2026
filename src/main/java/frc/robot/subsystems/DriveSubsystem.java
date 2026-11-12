package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends SubsystemBase {

  private static DriveSubsystem s_driveSubsystem;

  Pigeon2 m_gyro;

  TalonFX m_leftFrontDriveMotor;
  TalonFX m_rightFrontDriveMotor;
  TalonFX m_leftBackDriveMotor;
  TalonFX m_rightBackDriveMotor;

  DifferentialDrivetrainSim m_DifferentialDrivetrainSim;

  DifferentialDrivePoseEstimator m_poseEstimator;

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
    m_gyro = new Pigeon2(0);

    m_leftFrontDriveMotor = new TalonFX(10);
    m_rightFrontDriveMotor = new TalonFX(11);
    m_leftBackDriveMotor = new TalonFX(12);
    m_rightBackDriveMotor = new TalonFX(13);

    m_leftBackDriveMotor.setControl(new Follower(10, MotorAlignmentValue.Aligned));
    m_rightBackDriveMotor.setControl(new Follower(11, MotorAlignmentValue.Aligned));

    m_DifferentialDrivetrainSim =
        new DifferentialDrivetrainSim(
            DCMotor.getKrakenX60(2),
            10.71,
            3.5,
            18.35,
            Units.inchesToMeters(3),
            Units.inchesToMeters(21.67),
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

    m_poseEstimator =
        new DifferentialDrivePoseEstimator(
            new DifferentialDriveKinematics(Constants.Drive.DRIVE_TRACK_WIDTH.in(Meters)),
            Rotation2d.kZero,
            0,
            0,
            new Pose2d(0, 0, Rotation2d.kZero));

    m_robotDrive =
        new DifferentialDrive(
            (double speed) -> {
              m_leftFrontDriveMotor.set(speed);
            },
            (double speed) -> {
              m_rightFrontDriveMotor.set(-speed);
            });
    // 676767
  }

  @Override
  public void periodic() {
    double driveRequest = m_driveRequest.getAsDouble();
    double turnRequest = m_turnRequest.getAsDouble();
    m_robotDrive.arcadeDrive(driveRequest, turnRequest, true);

    Pose2d pose =
        m_poseEstimator.update(
            m_gyro.getRotation2d(),
            new DifferentialDriveWheelPositions(
                m_leftFrontDriveMotor.getPosition().getValue().in(Rotations)
                    * Constants.Drive.DRIVE_GEAR_RATIO
                    / Constants.Drive.DRIVE_WHEEL_CIRCUMFERENCE.in(Meters),
                m_rightFrontDriveMotor.getPosition().getValue().in(Rotations)
                    * Constants.Drive.DRIVE_GEAR_RATIO
                    / Constants.Drive.DRIVE_WHEEL_CIRCUMFERENCE.in(Meters)));

    Logger.recordOutput("DriveSubsystem/pose", pose);
    Logger.recordOutput("DriveSubsystem/driveRequest", m_driveRequest.getAsDouble());
    Logger.recordOutput("DriveSubsystem/turnRequest", m_turnRequest.getAsDouble());
    DifferentialDrive.WheelSpeeds wheelSpeeds = DifferentialDrive.arcadeDriveIK(0, 0, true);
    Logger.recordOutput("DriveSubsystem/leftWheelWantedOutput", wheelSpeeds.left);
    Logger.recordOutput("DriveSubsystem/rightWheelWantedOutput", wheelSpeeds.right);
    Logger.recordOutput(
        "DriveSubsystem/leftWheelActualDutyCycle", m_leftFrontDriveMotor.getDutyCycle().getValue());
    Logger.recordOutput(
        "DriveSubsystem/rightWheelActualDutyCycle",
        -m_rightFrontDriveMotor.getDutyCycle().getValue());
  }

  public void simulationPeriodic() {
    var leftFrontDriveMotorSim = m_leftFrontDriveMotor.getSimState();
    var rightFrontDriveMotorSim = m_rightFrontDriveMotor.getSimState();

    leftFrontDriveMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;
    rightFrontDriveMotorSim.Orientation = ChassisReference.Clockwise_Positive;

    leftFrontDriveMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    rightFrontDriveMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    var leftFrontDriveMotorSimVoltage = leftFrontDriveMotorSim.getMotorVoltage();
    var rightFrontDriveMotorSimVoltage = leftFrontDriveMotorSim.getMotorVoltage();
    m_DifferentialDrivetrainSim.setInputs(
        leftFrontDriveMotorSimVoltage, rightFrontDriveMotorSimVoltage);

    m_DifferentialDrivetrainSim.update(0.02);

    leftFrontDriveMotorSim.setRawRotorPosition(
        m_DifferentialDrivetrainSim.getLeftPositionMeters()
            / Constants.Drive.DRIVE_WHEEL_CIRCUMFERENCE.in(Meters)
            * Constants.Drive.DRIVE_GEAR_RATIO);
    leftFrontDriveMotorSim.setRotorVelocity(
        m_DifferentialDrivetrainSim.getLeftVelocityMetersPerSecond()
            / Constants.Drive.DRIVE_WHEEL_CIRCUMFERENCE.in(Meters)
            * Constants.Drive.DRIVE_GEAR_RATIO);
    rightFrontDriveMotorSim.setRawRotorPosition(
        m_DifferentialDrivetrainSim.getRightPositionMeters()
            / Constants.Drive.DRIVE_WHEEL_CIRCUMFERENCE.in(Meters)
            * Constants.Drive.DRIVE_GEAR_RATIO);
    rightFrontDriveMotorSim.setRotorVelocity(
        m_DifferentialDrivetrainSim.getRightVelocityMetersPerSecond()
            / Constants.Drive.DRIVE_WHEEL_CIRCUMFERENCE.in(Meters)
            * Constants.Drive.DRIVE_GEAR_RATIO);
  }
}
