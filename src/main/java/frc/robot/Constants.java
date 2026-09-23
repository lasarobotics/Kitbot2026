package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public final class Constants {
  public static class FuelController {
    public static final AngularVelocity SHOOTING_SHOOTER_SPEED = RotationsPerSecond.of(40);
    public static final AngularVelocity SHOOTING_INDEXER_SPEED = RotationsPerSecond.of(-20);

    public static final AngularVelocity INTAKING_SHOOTER_SPEED = RotationsPerSecond.of(30);
    public static final AngularVelocity INTAKING_INDEXER_SPEED = RotationsPerSecond.of(40);

    public static final AngularVelocity REVERSING_SHOOTER_SPEED = RotationsPerSecond.of(-30);
    public static final AngularVelocity REVERSING_INDEXER_SPEED = RotationsPerSecond.of(-20);

    public static final AngularVelocity SHOOTER_SPEED_ERROR_TOLERANCE = RotationsPerSecond.of(2);
  }
}