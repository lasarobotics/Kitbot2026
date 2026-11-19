package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends SubsystemBase {

  private static DriveSubsystem s_driveSubsystem;
  Pigeon2 m_gyro;
  TalonFX m_leftFrontDriveMotor;
  TalonFX m_rightFrontDriveMotor;
  TalonFX m_leftBackDriveMotor;
  TalonFX m_rightBackDriveMotor;

  DifferentialDrivetrainSim m_differentialDrivetrainSim;
  DifferentialDrivePoseEstimator m_poseEstimator;
  DifferentialDrive m_robotDrive;

  DoubleSupplier m_driveRequest;
  DoubleSupplier m_turnRequest;
  BooleanSupplier m_zeroRequest;

  private static volatile double s_limelightHeartbeat;

  public static DriveSubsystem getInstance() {
    if (s_driveSubsystem == null) {
      s_driveSubsystem = new DriveSubsystem();
    }
    return s_driveSubsystem;
  }

  public void configureBindings(
      DoubleSupplier driveRequest, DoubleSupplier turnRequest, BooleanSupplier zeroRequest) {
    m_driveRequest = driveRequest;
    m_turnRequest = turnRequest;
    m_zeroRequest = zeroRequest;
  }

  public DriveSubsystem() {
    m_gyro = new Pigeon2(9);
    m_leftFrontDriveMotor = new TalonFX(10);
    m_rightFrontDriveMotor = new TalonFX(11);
    m_leftBackDriveMotor = new TalonFX(12);
    m_rightBackDriveMotor = new TalonFX(13);

    TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
    leftMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
    rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    m_leftFrontDriveMotor.getConfigurator().apply(leftMotorConfig);
    m_rightFrontDriveMotor.getConfigurator().apply(rightMotorConfig);

    m_leftBackDriveMotor.setControl(new Follower(10, MotorAlignmentValue.Aligned));
    m_rightBackDriveMotor.setControl(new Follower(11, MotorAlignmentValue.Aligned));

    m_differentialDrivetrainSim =
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
            new DifferentialDriveKinematics(Constants.Drive.DRIVETRAIN_TRACK_WIDTH.in(Meters)),
            Rotation2d.kZero,
            0,
            0,
            new Pose2d(0, 0, Rotation2d.kZero));

    m_robotDrive = new DifferentialDrive(m_leftFrontDriveMotor::set, m_rightFrontDriveMotor::set);
  }

  @Override
  public void periodic() {

    LimelightHelpers.PoseEstimate poseEstimate = getFilteredLimelightPose();

    if (poseEstimate != null) {
      if (DriverStation.isDisabled()) {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.25, 0.25, 0.25));
      } else {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 999999));
      }

      m_poseEstimator.addVisionMeasurement(
          poseEstimate.pose, Utils.fpgaToCurrentTime(poseEstimate.timestampSeconds));
    }

    double driveRequest = m_driveRequest.getAsDouble();
    double turnRequest = m_turnRequest.getAsDouble();
    boolean zeroRequest = m_zeroRequest.getAsBoolean();

    m_robotDrive.arcadeDrive(driveRequest, turnRequest, true);

    if (zeroRequest) {
      m_gyro.reset();
    }

    m_poseEstimator.update(
        m_gyro.getRotation2d(),
        new DifferentialDriveWheelPositions(
            (m_leftFrontDriveMotor.getPosition().getValue().in(Rotations)
                / Constants.Drive.DRIVETRAIN_GEAR_RATIO
                * Constants.Drive.DRIVETRAIN_WHEEL_CIRCUMFERENCE.in(Meters)),
            (m_rightFrontDriveMotor.getPosition().getValue().in(Rotations)
                    / Constants.Drive.DRIVETRAIN_GEAR_RATIO)
                * Constants.Drive.DRIVETRAIN_WHEEL_CIRCUMFERENCE.in(Meters)));

    Pose2d pose = m_poseEstimator.getEstimatedPosition();

    // limelight hub orientation

    Logger.recordOutput("DriveSubsystem/zeroRequest", m_zeroRequest.getAsBoolean());
    Logger.recordOutput("DriveSubsystem/pose", pose);
    Logger.recordOutput("DriveSubsystem/driveRequest", m_driveRequest.getAsDouble());
    Logger.recordOutput("DriveSubsystem/turnRequest", m_turnRequest.getAsDouble());
    DifferentialDrive.WheelSpeeds wheelSpeeds =
        DifferentialDrive.arcadeDriveIK(driveRequest, turnRequest, true);
    Logger.recordOutput("DriveSubsystem/leftWheelWantedOutput", wheelSpeeds.left);
    Logger.recordOutput("DriveSubsystem/rightWheelWantedOutput", wheelSpeeds.right);
    Logger.recordOutput(
        "DriveSubsystem/leftWheelActualDutyCycle", m_leftFrontDriveMotor.getDutyCycle().getValue());
    Logger.recordOutput(
        "DriveSubsystem/rightWheelActualDutyCycle",
        m_rightFrontDriveMotor.getDutyCycle().getValue());
  }

  private LimelightHelpers.PoseEstimate getFilteredLimelightPose() {

    LimelightHelpers.PoseEstimate pose_estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("");

    double hb = LimelightHelpers.getHeartbeat("");

    if (hb == s_limelightHeartbeat) {
      return null;
    } else {
      s_limelightHeartbeat = hb;
    }

    if (pose_estimate == null) {
      return null;
    }

    if (m_gyro.getAngularVelocityZWorld().getValue().abs(RotationsPerSecond) > 2 * Math.PI) {
      return null;
    }

    if (pose_estimate.tagCount == 0) {
      return null;
    }

    if (Double.isNaN(pose_estimate.pose.getX())
        || Double.isNaN(pose_estimate.pose.getX())
        || Double.isNaN(pose_estimate.pose.getRotation().getDegrees())) {
      return null;
    }

    if (pose_estimate.pose.getX() < 0
        || pose_estimate.pose.getX() > Constants.Feild.FIELD_X.in(Meters)
        || pose_estimate.pose.getY() < 0
        || pose_estimate.pose.getY() > Constants.Feild.FIELD_Y.in(Meters)) {

      return null;
    }

    if (pose_estimate.tagCount == 1 && pose_estimate.rawFiducials.length == 1) {
      RawFiducial tag = pose_estimate.rawFiducials[0];
      if (tag.ambiguity > Constants.Limelight_Constants.SINGLE_TAG_AMBIGUITY_CUTOFF) {
        return null;
      }
      if (tag.distToCamera > Constants.Limelight_Constants.SINGLE_TAG_DISTANCE_CUTOFF) {
        return null;
      }
    }

    return pose_estimate;
  }

  public void simulationPeriodic() {
    var leftFrontDriveMotorSim = m_leftFrontDriveMotor.getSimState();
    var rightFrontDriveMotorSim = m_rightFrontDriveMotor.getSimState();
    var gyroSim = m_gyro.getSimState();

    leftFrontDriveMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;
    rightFrontDriveMotorSim.Orientation = ChassisReference.Clockwise_Positive;

    leftFrontDriveMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    rightFrontDriveMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    var leftFrontDriveMotorVoltage = leftFrontDriveMotorSim.getMotorVoltageMeasure();
    var rightFrontDriveMotorVoltage = rightFrontDriveMotorSim.getMotorVoltageMeasure();
    m_differentialDrivetrainSim.setInputs(
        leftFrontDriveMotorVoltage.in(Volts), rightFrontDriveMotorVoltage.in(Volts));

    m_differentialDrivetrainSim.update(0.02);

    leftFrontDriveMotorSim.setRawRotorPosition(
        (m_differentialDrivetrainSim.getLeftPositionMeters()
                / Constants.Drive.DRIVETRAIN_WHEEL_CIRCUMFERENCE.in(Meters))
            * Constants.Drive.DRIVETRAIN_GEAR_RATIO);
    leftFrontDriveMotorSim.setRotorVelocity(
        (m_differentialDrivetrainSim.getLeftVelocityMetersPerSecond()
                / Constants.Drive.DRIVETRAIN_WHEEL_CIRCUMFERENCE.in(Meters))
            * Constants.Drive.DRIVETRAIN_GEAR_RATIO);
    rightFrontDriveMotorSim.setRawRotorPosition(
        (m_differentialDrivetrainSim.getRightPositionMeters()
                / Constants.Drive.DRIVETRAIN_WHEEL_CIRCUMFERENCE.in(Meters))
            * Constants.Drive.DRIVETRAIN_GEAR_RATIO);
    rightFrontDriveMotorSim.setRotorVelocity(
        (m_differentialDrivetrainSim.getRightVelocityMetersPerSecond()
                / Constants.Drive.DRIVETRAIN_WHEEL_CIRCUMFERENCE.in(Meters))
            * Constants.Drive.DRIVETRAIN_GEAR_RATIO);
    gyroSim.setRawYaw(m_differentialDrivetrainSim.getHeading().getDegrees());
  }
}
