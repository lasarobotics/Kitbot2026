package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  DifferentialDrivetrainSim m_DifferentialDrivetrainSim;

  DifferentialDrivePoseEstimator m_poseEstimator;

  // to do :D gotta update kp ki and kd when phoenix tuner available
  PIDController pid;

  public double TargetX;
  public double TargetY;

  DifferentialDrive m_robotDrive;

  DoubleSupplier m_driveRequest;
  DoubleSupplier m_turnRequest;
  BooleanSupplier m_zeroRequest;
  BooleanSupplier m_agitateRequest;
  
  Timer m_agitationFlipTimer = new Timer();

  public static DriveSubsystem getInstance() {
    if (s_driveSubsystem == null) {
      s_driveSubsystem = new DriveSubsystem();
    }
    return s_driveSubsystem;
  }

  public void configureBindings(
      DoubleSupplier driveRequest, DoubleSupplier turnRequest, BooleanSupplier zeroRequest, BooleanSupplier agitateRequest) {
    m_driveRequest = driveRequest;
    m_turnRequest = turnRequest;
    m_zeroRequest = zeroRequest;
    m_agitateRequest = agitateRequest;
    }

  public DriveSubsystem() {
    m_gyro = new Pigeon2(0);

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
            new DifferentialDriveKinematics(Constants.Drive.DRIVETRAIN_TRACK_WIDTH.in(Meters)),
            Rotation2d.kZero,
            0,
            0,
            new Pose2d(0, 0, Rotation2d.kZero));

    pid = new PIDController(0, 0, 0);
    pid.enableContinuousInput(-180, 180);
    pid.setIntegratorRange(-.5, .5);
    pid.setIZone(2);

    m_robotDrive = new DifferentialDrive(m_leftFrontDriveMotor::set, m_rightFrontDriveMotor::set);
    // 676767
  }

  @Override
  public void periodic() {
    double driveRequest = m_driveRequest.getAsDouble();
    double turnRequest = m_turnRequest.getAsDouble();
    boolean agitateRequest = m_agitateRequest.getAsBoolean();
    boolean zeroRequest = m_zeroRequest.getAsBoolean();

    m_robotDrive.arcadeDrive(driveRequest, turnRequest, true);

    if (zeroRequest) {
      if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)) {
        m_poseEstimator.resetRotation(Rotation2d.kZero);
        TargetX = 4.619;
        TargetY = 4.049;
      } else {
        m_poseEstimator.resetRotation(Rotation2d.k180deg);
        TargetX = 11.925;
        TargetY = 4.049;
      }
    }

    if (agitateRequest) {
      if (m_agitationFlipTimer.get() == 0) {
        m_agitationFlipTimer.start();
      } 
      if (m_agitationFlipTimer.hasElapsed(Constants.Drive.AGITATION_FLIP_TIME)) {
        m_robotDrive.arcadeDrive(1, 0, true); // Moves forward
      } else {
        m_robotDrive.arcadeDrive(-1, 0, true); // Moves backward
      }
      if (m_agitationFlipTimer.hasElapsed(Constants.Drive.AGITATION_FLIP_TIME.times(2).in(Seconds))) {
        m_agitationFlipTimer.reset();
      }
    }

    m_poseEstimator.update(
        m_gyro.getRotation2d(),
        new DifferentialDriveWheelPositions(
            m_leftFrontDriveMotor.getPosition().getValue().in(Rotations)
                / Constants.Drive.DRIVETRAIN_GEAR_RATIO
                * Constants.Drive.DRIVETRAIN_WHEEL_CIRCUMFERENCE.in(Meters),
            m_rightFrontDriveMotor.getPosition().getValue().in(Rotations)
                / Constants.Drive.DRIVETRAIN_GEAR_RATIO
                * Constants.Drive.DRIVETRAIN_WHEEL_CIRCUMFERENCE.in(Meters)));

    Pose2d pose = m_poseEstimator.getEstimatedPosition();

    // to do :D gotta find the target x and y to then calc y error and x error for the set point
    MathUtil.clamp(
        pid.calculate(
            pose.getRotation().getRotations(),
            Math.atan((TargetY - pose.getY()) / (TargetX - pose.getX()))),
        -1,
        1);

    Logger.recordOutput("DriveSubsystem/pose", pose);
    Logger.recordOutput("DriveSubsystem/driveRequest", m_driveRequest.getAsDouble());
    Logger.recordOutput("DriveSubsystem/turnRequest", m_turnRequest.getAsDouble());
    Logger.recordOutput("DriveSubsystem/zeroRequest", m_zeroRequest.getAsBoolean());
    Logger.recordOutput("DriveSubsystem/agitateRequest", m_agitateRequest.getAsBoolean());
    Logger.recordOutput("DriveSubsystem/agitationFlipTimer", m_agitationFlipTimer.get());

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
            / Constants.Drive.DRIVETRAIN_WHEEL_CIRCUMFERENCE.in(Meters)
            * Constants.Drive.DRIVETRAIN_GEAR_RATIO);
    leftFrontDriveMotorSim.setRotorVelocity(
        m_DifferentialDrivetrainSim.getLeftVelocityMetersPerSecond()
            / Constants.Drive.DRIVETRAIN_WHEEL_CIRCUMFERENCE.in(Meters)
            * Constants.Drive.DRIVETRAIN_GEAR_RATIO);
    rightFrontDriveMotorSim.setRawRotorPosition(
        m_DifferentialDrivetrainSim.getRightPositionMeters()
            / Constants.Drive.DRIVETRAIN_WHEEL_CIRCUMFERENCE.in(Meters)
            * Constants.Drive.DRIVETRAIN_GEAR_RATIO);
    rightFrontDriveMotorSim.setRotorVelocity(
        m_DifferentialDrivetrainSim.getRightVelocityMetersPerSecond()
            / Constants.Drive.DRIVETRAIN_WHEEL_CIRCUMFERENCE.in(Meters)
            * Constants.Drive.DRIVETRAIN_GEAR_RATIO);
  }
}
