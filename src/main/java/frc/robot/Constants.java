package frc.robot;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A file to store all the constant values, offset, and other fixed numerical values and details of the robot
 */
public class Constants {

    public static final boolean blueAlliance = true;
    public static final boolean smartEnable = true;
    public static final double ControllerDeadband = 0.05;
    public static final boolean VisionOdometry = true;
    public static double MAX_SPEED = 5;
    public static double povSpeed = 0.5;

    public static final class IntakeConstants {
        public static final int INTAKE_ROLLER_ID = 26;
        public static final int INTAKE_PIVOT_ID  = 25;

        public static final double PIVOT_kP = 0.02;
        public static final double PIVOT_kI = 0.0;
        public static final double PIVOT_kD = 0;

        public static final double PIVOT_GEAR_RATIO    = 400.0;

        public static final double ROLLER_INTAKE_SPEED  =  0.9;
        public static final double ROLLER_OUTTAKE_SPEED = -0.9;

        public static final double PivotMaxSpeed = 0.15;
    }

    public static final class FeederConstants {
        public static final int FEEDER_ID = 27;

        public static final double FEEDER_SPEED         = -0.35;
        public static final double FEEDER_REVERSE_SPEED = 0.35;
    }

    public static final class ShooterConstants {
        public static final int SHOOTER_1_ID = 21;
        public static final int SHOOTER_2_ID = 22;
        public static final int SHOOTER_3_ID = 23;
        public static final int SHOOTER_4_ID = 24;

        public static final double SHOOTER_kP = 0.22;
        public static final double SHOOTER_kI = 0.0001;
        public static final double SHOOTER_kD = 0.01;
        public static final double SHOOTER_kV = 0.0957;
        public static final double SHOOTER_kS = 0.0;

        public static final double SHOOT_VELOCITY_RPS   =  47.0;
        public static final double REVERSE_VELOCITY_RPS = -53.0;
        public static final double SHOOTER_4_INTAKE_RPS =  100.0;

        public static final double SHOOTER_SUPPLY_LIMIT = 60;
        public static final double SHOOTER_STATOR_LIMIT = 80;

        public static final double VELOCITY_TOLERANCE_RPS = 5.0;
    

        //  Distance → RPS Lookup Table
        // Key   = distance from target in METERS
        // Value = shooter flywheel speed in RPS
        public static final InterpolatingDoubleTreeMap SHOOTER_MAP =
            new InterpolatingDoubleTreeMap();

        static {
            SHOOTER_MAP.put(0.0, 45.0);
            SHOOTER_MAP.put(1.75, 55.0);
            SHOOTER_MAP.put(1.95, 57.0);
            SHOOTER_MAP.put(2.5, 60.0);
            SHOOTER_MAP.put(3.0, 63.0);
            SHOOTER_MAP.put(3.25, 66.0);
            SHOOTER_MAP.put(3.5, 67.0);
            SHOOTER_MAP.put(4.0, 70.0);
            SHOOTER_MAP.put(200.0,100.0);
        }
    }

    public static class FieldPoses{
        public static final Translation2d Hub = blueAlliance ? new Translation2d(4.625,4.035) : new Translation2d(11.920,4.035);
    }
}