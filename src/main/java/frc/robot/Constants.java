package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public final class Constants {
  public static class FuelController {
    public static final AngularVelocity SHOOTING_SHOOTER_SPEED = RotationsPerSecond.of(55);
    public static final AngularVelocity SHOOTING_INDEXER_SPEED = RotationsPerSecond.of(-30);

    public static final AngularVelocity INTAKING_SHOOTER_SPEED = RotationsPerSecond.of(40);
    public static final AngularVelocity INTAKING_INDEXER_SPEED = RotationsPerSecond.of(100);

    public static final AngularVelocity REVERSING_SHOOTER_SPEED = RotationsPerSecond.of(-30);
    public static final AngularVelocity REVERSING_INDEXER_SPEED = RotationsPerSecond.of(-35);

    public static final AngularVelocity SHOOTER_SPEED_ERROR_TOLERANCE = RotationsPerSecond.of(2);
  }

  public static class Drive {
    public static final double DRIVETRAIN_GEAR_RATIO = 10.71;
    public static final Distance DRIVETRAIN_WHEEL_CIRCUMFERENCE = Inches.of(6 * Math.PI);
    public static final Distance DRIVETRAIN_TRACK_WIDTH = Inches.of(21.58);
  }

  public static class Auto {
    public static final Time AUTO_DRIVE_TIME = Seconds.of(.85);
    public static final Time AUTO_SHOOT_TIME = Seconds.of(4);
    // time of auto turn needs to be determined with testing so it is at some certain angle;
    // maybe 180*, or tilted to a trench
    public static final Time AUTO_TURN_TIME = Seconds.of(0.67);
    public static final Time TOTAL_AUTO_TIME =
        AUTO_DRIVE_TIME.plus(AUTO_SHOOT_TIME).plus(AUTO_TURN_TIME);

    public static final double AUTO_DRIVE_SPEED = 0.5;
  }

  public static class Field {
    public static final Distance FIELD_X = Inches.of(650.12);
    public static final Distance FIELD_Y = Inches.of(316.64);
  }

  public static class Limelight_Constants {
    public static final double SINGLE_TAG_DISTANCE_CUTOFF = 0.5;
    public static final double SINGLE_TAG_AMBIGUITY_CUTOFF = 5;
  }
}
