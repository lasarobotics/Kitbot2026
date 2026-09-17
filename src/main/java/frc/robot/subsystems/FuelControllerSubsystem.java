package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;

public class FuelControllerSubsystem extends SubsystemBase {

  public static FuelControllerSubsystem s_fuelControllerSubsystem;
  TalonFX m_shooterIntakeMotor;
  TalonFX m_indexerMotor;

  private final VelocityVoltage m_shooterIntakeRequest;
  private final VelocityVoltage m_indexerRequest;

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
  }

  public void configureBindings(
      BooleanSupplier shouldShoot, BooleanSupplier shouldIntake, BooleanSupplier shouldReverse) {
    m_shouldShoot = shouldShoot;
    m_shouldIntake = shouldIntake;
    m_shouldReverse = shouldReverse;
  }

  @Override
  public void periodic() {

    if (m_shouldShoot.getAsBoolean()) {
      runShooter();

      if (atShootingSpeed()) {
        runIndexer();
      } else {
        stopIndexer();
      }

    } else if (m_shouldIntake.getAsBoolean()) {
      setMotorsIntake();
    } else if (m_shouldReverse.getAsBoolean()) {
      setMotorsReverse();
    } else {
      stopMotors();
    }
    // 676767
  }

  private boolean atShootingSpeed() {
    return m_shooterIntakeMotor
        .getVelocity()
        .isNear(
            Constants.FuelController.SHOOTING_SHOOTER_SPEED,
            Constants.FuelController.SHOOTER_SPEED_ERROR_TOLERANCE);
  }

  private void runShooter() {
    m_shooterIntakeMotor.setControl(
        m_shooterIntakeRequest.withVelocity(Constants.FuelController.SHOOTING_SHOOTER_SPEED));
  }

  private void runIndexer() {
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
        m_shooterIntakeRequest.withVelocity(Constants.FuelController.REVERSING_SHOOTER_SPEED));
    m_indexerMotor.setControl(
        m_indexerRequest.withVelocity(Constants.FuelController.REVERSING_INDEXER_SPEED));
  }

  private void stopMotors() {
    stopShooter();
    stopIndexer();
  }

  private void stopShooter() {
    m_shooterIntakeMotor.stopMotor();
  }

  private void stopIndexer() {
    m_indexerMotor.stopMotor();
  }
}
