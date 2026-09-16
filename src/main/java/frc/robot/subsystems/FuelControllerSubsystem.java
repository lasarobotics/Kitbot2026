package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelControllerSubsystem extends SubsystemBase {

  private static FuelControllerSubsystem s_fuelControllerSubsystem;
  TalonFX m_leftMainMotor;
  TalonFX m_rightMainMotor;

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
    m_leftMainMotor = new TalonFX(30);
    m_rightMainMotor = new TalonFX(31);
  }

  public void configureBindings(
    BooleanSupplier shouldShoot, BooleanSupplier shouldIntake, BooleanSupplier shouldReverse) {
      m_shouldShoot = shouldShoot;
      m_shouldReverse = shouldReverse;
      m_shouldIntake = shouldIntake;
    }
  

  @Override
  public void periodic() {
    if (m_shouldShoot.getAsBoolean()) {
      m_leftMainMotor.set(1.0);
      m_rightMainMotor.set(-1.0);
    } else if (m_shouldIntake.getAsBoolean()) {
      m_leftMainMotor.set(1.0);
      m_rightMainMotor.set(1.0);
    } else if (m_shouldReverse.getAsBoolean()) {
      m_leftMainMotor.set(-1.0);
      m_rightMainMotor.set(-1.0);
    } else {
      m_leftMainMotor.set(0.0);
      m_rightMainMotor.set(0.0);
    }
    }
  }

