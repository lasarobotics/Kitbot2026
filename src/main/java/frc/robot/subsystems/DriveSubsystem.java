package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.VecBuilder;
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

  DifferentialDrivetrainSim m_differentialDrivetrainSim;

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

    m_differentialDrivetrainSim =
        new DifferentialDrivetrainSim(
            DCMotor.getKrakenX60(2),
            10.71,
            3.5,
            18.35,
            Units.inchesToMeters(3),
            Units.inchesToMeters(21.67),
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

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
                / Constants.Drive.DRIVETRAIN_WHEEL_DIAMETER.in(Meters))
            * Constants.Drive.DRIVETRAIN_GEAR_RATIO);
    leftFrontDriveMotorSim.setRotorVelocity(
        (m_differentialDrivetrainSim.getLeftVelocityMetersPerSecond()
                / Constants.Drive.DRIVETRAIN_WHEEL_DIAMETER.in(Meters))
            * Constants.Drive.DRIVETRAIN_GEAR_RATIO);
    rightFrontDriveMotorSim.setRawRotorPosition(
        (m_differentialDrivetrainSim.getRightPositionMeters()
                / Constants.Drive.DRIVETRAIN_WHEEL_DIAMETER.in(Meters))
            * Constants.Drive.DRIVETRAIN_GEAR_RATIO);
    rightFrontDriveMotorSim.setRotorVelocity(
        (m_differentialDrivetrainSim.getRightVelocityMetersPerSecond()
                / Constants.Drive.DRIVETRAIN_WHEEL_DIAMETER.in(Meters))
            * Constants.Drive.DRIVETRAIN_GEAR_RATIO);
    gyroSim.setRawYaw(m_differentialDrivetrainSim.getHeading().getDegrees());
  }
}
