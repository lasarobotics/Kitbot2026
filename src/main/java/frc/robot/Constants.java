package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

public class Constants {
  public static class FuelController {
    public static final AngularVelocity SHOOTING_SHOOTER_SPEED = RotationsPerSecond.of(40);
    public static final AngularVelocity SHOOTING_INDEXER_SPEED = RotationsPerSecond.of(-20);

    public static final AngularVelocity INTAKING_SHOOTER_SPEED = RotationsPerSecond.of(30);
    public static final AngularVelocity INTAKING_INDEXER_SPEED = RotationsPerSecond.of(40);

    public static final AngularVelocity REVERSING_SHOOTER_SPEED = RotationsPerSecond.of(-30);
    public static final AngularVelocity REVERSING_INDEXER_SPEED = RotationsPerSecond.of(-30);

    public static final AngularVelocity SHOOTER_SPEED_ERROR_TOLERANCE = RotationsPerSecond.of(2);
  }

  public static class Auto {
    public static final Time AUTO_DRIVE_TIME = Seconds.of(2);
    public static final Time AUTO_SHOOT_TIME = Seconds.of(2);
    public static final Time TOTAL_AUTO_TIME = AUTO_DRIVE_TIME.plus(AUTO_SHOOT_TIME);

    public static final double AUTO_DRIVE_SPEED = 0.5;
  }
}
