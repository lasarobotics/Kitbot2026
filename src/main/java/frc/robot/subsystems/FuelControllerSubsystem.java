package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class FuelControllerSubsystem extends SubsystemBase {

  private static FuelControllerSubsystem s_fuelControllerSubsystem;
  TalonFX m_shooterIntakeMotor;
  TalonFX m_indexerMotor;

  VelocityVoltage m_shooterIntakeRequest;
  VelocityVoltage m_indexerRequest;

  BooleanSupplier m_shouldShoot;
  BooleanSupplier m_shouldIntake;
  BooleanSupplier m_shouldReverse;

  public static FuelControllerSubsystem getInstance() {
    if (s_fuelControllerSubsystem == null) {
      s_fuelControllerSubsystem = new FuelControllerSubsystem();
    }
    return s_fuelControllerSubsystem;
  }

  public FuelControllerSubsystem() {
    m_shooterIntakeMotor = new TalonFX(30);
    m_indexerMotor = new TalonFX(31);

    m_shooterIntakeRequest = new VelocityVoltage(0);
    m_indexerRequest = new VelocityVoltage(0);

    TalonFXConfiguration shooterIntakeConfig = new TalonFXConfiguration();
    TalonFXConfiguration indexerConfig = new TalonFXConfiguration();

    m_shooterIntakeMotor.getConfigurator().apply(shooterIntakeConfig);
    m_indexerMotor.getConfigurator().apply(indexerConfig);
  }

  public void configureBindings(
      BooleanSupplier shouldShoot, BooleanSupplier shouldIntake, BooleanSupplier shouldReverse) {
    m_shouldShoot = shouldShoot;
    m_shouldIntake = shouldIntake;
    m_shouldReverse = shouldReverse;
  }

  @Override
  public void periodic() {
    boolean shooting = m_shouldShoot.getAsBoolean();
    boolean intaking = m_shouldIntake.getAsBoolean();
    boolean reversing = m_shouldReverse.getAsBoolean();

    if (shooting) {
      runShooterMotor();
      if (atShootingSpeed()) {
        runIndexerMotor();
      } else {
        stopIndexer();
      }
    } else if (intaking) {
      setMotorsIntake();
    } else if (reversing) {
      // reversing
      setMotorsReverse();
    } else {
      // nothing
      stopMotors();
    }

    Logger.recordOutput("FuelControllerSubsystem/atShootingSpeed", atShootingSpeed());
    Logger.recordOutput(
        "FuelControllerSubsystem/shooterIntakeSpeed",
        m_shooterIntakeMotor.getVelocity().getValue().in(RotationsPerSecond));
    Logger.recordOutput(
        "FuelControllerSubsystem/indexerSpeed",
        m_indexerMotor.getVelocity().getValue().in(RotationsPerSecond));
    if (shooting) {
      Logger.recordOutput("FuelControllerSubsystem/currentState", "shooting");
    } else if (intaking) {
      Logger.recordOutput("FuelControllerSubsystem/currentState", "intaking");
    } else if (reversing) {
      Logger.recordOutput("FuelControllerSubsystem/currentState", "reversing");
    } else {
      Logger.recordOutput("FuelControllerSubsystem/currentState", "nothing");
    }
  }

  private boolean atShootingSpeed() {
    return m_shooterIntakeMotor
        .getVelocity()
        .isNear(
            Constants.FuelController.SHOOTING_SHOOTER_SPEED,
            Constants.FuelController.SHOOTER_SPEED_ERROR_TOLERANCE);
  }

  private void runShooterMotor() {
    m_shooterIntakeMotor.setControl(
        m_shooterIntakeRequest.withVelocity(Constants.FuelController.SHOOTING_SHOOTER_SPEED));
  }

  private void runIndexerMotor() {
    m_indexerMotor.setControl(
        m_indexerRequest.withVelocity(Constants.FuelController.SHOOTING_INDEXER_SPEED));
  }

  private void setMotorsIntake() {
    m_shooterIntakeMotor.setControl(
        m_shooterIntakeRequest.withVelocity(Constants.FuelController.INTAKING_SHOOTER_SPEED));
    m_indexerMotor.setControl(
        m_indexerRequest.withVelocity(Constants.FuelController.INTAKING_INDEXER_SPEED));
  }

  private void setMotorsReverse() {
    m_shooterIntakeMotor.setControl(
        m_shooterIntakeRequest.withVelocity(Constants.FuelController.REVERSING_INDEXER_SPEED));
    m_indexerMotor.setControl(
        m_indexerRequest.withVelocity(Constants.FuelController.REVERSING_INDEXER_SPEED));
  }

  private void stopMotors() {
    stopShooter();
    stopIndexer();
  }

  private void stopShooter() {
    m_shooterIntakeMotor.stopMotor();
    ;
  }

  private void stopIndexer() {
    m_indexerMotor.stopMotor();
  }
}
