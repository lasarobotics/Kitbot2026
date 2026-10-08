package frc.robot;

// import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

// import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

public final class Constants {
  public static class FuelController {
    public static final double SHOOTING_SHOOTER_SPEED = 2.0;
    public static final double SHOOTING_INDEXER_SPEED = -2.0;

    public static final double INTAKING_SHOOTER_SPEED = 2.0;
    public static final double INTAKING_INDEXER_SPEED = 2.3;

    public static final double REVERSING_SHOOTER_SPEED = -2.0;
    public static final double REVERSING_INDEXER_SPEED = -1.5;

    public static final double SHOOTER_SPEED_ERROR_TOLERANCE = 0.25;
  }

  public static class Auto {
    public static final Time AUTO_DRIVE_TIME = Seconds.of(2);
    public static final Time AUTO_SHOOT_TIME = Seconds.of(2);
    public static final Time TOTAL_AUTO_TIME = AUTO_DRIVE_TIME.plus(AUTO_SHOOT_TIME);

    // driving forwards (shooter shoots out back of robot)
    public static final double AUTO_DRIVE_SPEED = 0.5;
  }
}
