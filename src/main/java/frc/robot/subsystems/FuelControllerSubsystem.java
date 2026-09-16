package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

public class FuelControllerSubsystem extends SubsystemBase {

  public static FuelControllerSubsystem s_FuelControllerSubsystem;
  TalonFX m_leftMainMotor;
  TalonFX m_rightMainMotor;

  BooleanSupplier m_shouldShoot;
  BooleanSupplier m_shouldIntake;
  BooleanSupplier m_shouldReverse;

  public static FuelControllerSubsystem getInstance() {
    if (s_FuelControllerSubsystem == null) {
      s_FuelControllerSubsystem = new FuelControllerSubsystem();
    }
    return s_FuelControllerSubsystem;
  }

  public FuelControllerSubsystem() {
    m_leftMainMotor = new TalonFX(30);
    m_rightMainMotor = new TalonFX(31);
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
      // shooting
      m_leftMainMotor.set(1.0);
      m_rightMainMotor.set(-1.0);
    } else if (m_shouldIntake.getAsBoolean()) {
      // intake
      m_leftMainMotor.set(1.0);
      m_rightMainMotor.set(1.0);
    } else if (m_shouldReverse.getAsBoolean()) {
      // reverse
      m_leftMainMotor.set(-1.0);
      m_rightMainMotor.set(-1.0);
    } else {
      m_leftMainMotor.set(0.0);
      m_rightMainMotor.set(0.0);
    }
  }
}
