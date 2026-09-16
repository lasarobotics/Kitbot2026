package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelControllerSubsystem extends SubsystemBase {

  public static FuelControllerSubsystem s_FuelControllerSubsystem;
  TalonFX m_leftMainMotor;
  TalonFX m_rightMainMotor;

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
}
