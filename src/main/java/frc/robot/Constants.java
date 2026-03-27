package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import swervelib.math.Matter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.Meter;

/**
 * A file to store all the constant values, offset, and other fixed numerical values and details of the robot
 */
public class Constants {

  public static final boolean blueAlliance = true;

  public static final double ROBOT_MASS = 70;
  public static final Matter CHASSIS = new Matter(new Translation3d(-0.2, 0, Units.inchesToMeters(2)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final boolean smartEnable = true;
  public static final double ControllerDeadband = 0.05;
  public static final boolean VisionOdometry = false;
  public static double MAX_SPEED = 5;

  public static final class DrivebaseConstants {
public static final double WHEEL_LOCK_TIME = 100; // seconds
  }

  public static final class IntakeConstants {
        public static final int INTAKE_ROLLER_ID = 26;
        public static final int INTAKE_PIVOT_ID  = 25;

        public static final double PIVOT_kP = 2.0;
        public static final double PIVOT_kI = 0.0;
        public static final double PIVOT_kD = 0.1;
        public static final double PIVOT_kG = 0.3;

        public static final double PIVOT_DEPLOYED_ROT  = 10.0;
        public static final double PIVOT_RETRACTED_ROT =  0.0;
        public static final double PIVOT_GEAR_RATIO    = 25.0;

        public static final double ROLLER_INTAKE_SPEED  =  0.6;
        public static final double ROLLER_OUTTAKE_SPEED = -0.6;

        public static final double ROLLER_SUPPLY_LIMIT = 40;
        public static final double PIVOT_SUPPLY_LIMIT  = 40;
        public static final double PIVOT_STATOR_LIMIT  = 60;
    }

    public static final class FeederConstants {
        public static final int FEEDER_ID = 27;

        public static final double FEEDER_SPEED         = -0.2;
        public static final double FEEDER_REVERSE_SPEED = 0.2;

        public static final double FEEDER_SUPPLY_LIMIT = 40;
        public static final double FEEDER_STATOR_LIMIT = 60;
    }

    public static final class ShooterConstants {
        public static final int SHOOTER_1_ID = 21;
        public static final int SHOOTER_2_ID = 22;
        public static final int SHOOTER_3_ID = 23;
        public static final int SHOOTER_4_ID = 24;

        public static final double SHOOTER_kP = 0.55;
        public static final double SHOOTER_kI = 0.0;
        public static final double SHOOTER_kD = 0.01;
        public static final double SHOOTER_kV = 0.0957;
        public static final double SHOOTER_kS = 0.0;

        public static final double SHOOT_VELOCITY_RPS   =  47.0;
        public static final double REVERSE_VELOCITY_RPS = -53.0;
        public static final double SHOOTER_4_INTAKE_RPS =  100.0;

        public static final double SHOOTER_SUPPLY_LIMIT = 60;
        public static final double SHOOTER_STATOR_LIMIT = 80;

        public static final double VELOCITY_TOLERANCE_RPS = 5.0;
    

        // ─── Distance → RPS Lookup Table ──────────────────────────────────────
        // Key   = distance from target in METERS  (measured from Limelight)
        // Value = shooter flywheel speed in RPS
        // ⚠️  Tune these values by shooting from known distances and recording
        //     what RPS consistently scores. Add/remove rows as needed.
        public static final InterpolatingDoubleTreeMap SHOOTER_MAP =
            new InterpolatingDoubleTreeMap();

        static {
            SHOOTER_MAP.put(0.0, 40.0);
            SHOOTER_MAP.put(1.18, 54.5);
            SHOOTER_MAP.put(1.39, 54.0);
            SHOOTER_MAP.put(1.63, 56.5);
            SHOOTER_MAP.put(2.44, 65.0);
            SHOOTER_MAP.put(3.27, 69.89);
            SHOOTER_MAP.put(20.0,100.0);
        }
    }

    public static class CV {
        public static final double kp = 3;
        public static final double kd = 2;
        public static final double secondaryKp = 1.3;
        public static final double secondaryKd = 0.2;
        public static final double MaxSpeed = 0.5;
    }
}