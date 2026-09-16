package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelControllerSubsystem extends SubsystemBase {

  public static FuelControllerSubsystem s_fuelControllerSubsystem;
  TalonFX m_leftMainMotor;
  TalonFX m_rightMainMotor;

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
}
