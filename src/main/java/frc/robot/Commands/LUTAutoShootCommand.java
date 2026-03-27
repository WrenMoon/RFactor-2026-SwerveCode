package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
/**
 * LUTAutoShootCommand
 *
 * Triangle held:
 *   - Flywheel (shooter1/2/3) spins up immediately to ManualRPS from Shuffleboard
 *   - Shooter4 (column/CAN 24) starts after 2 second delay
 *   - No intake, no feeder involved here
 *
 * R2 held (separate binding in RobotContainer):
 *   - Feeder runs to push ball into flywheel
 */
import frc.robot.Subsystems.SwerveSubsystem;
public class LUTAutoShootCommand extends Command {

    private static final double COLUMN_DELAY_SEC = 2.0;

    private final ShooterSubsystem   shooter;

    private double targetRPS  = 67.0;

    public LUTAutoShootCommand( ShooterSubsystem shooter)
    {
        this.shooter   = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {

        // Flywheel spins immediately, column stays stopped
        //targetRPS = SmartDashboard.getNumber("AutoShoot/ManualRPS", 0.0);
        shooter.setVelocity(targetRPS);
        shooter.stop1(); // make sure column is off at start

        SmartDashboard.putString("AutoShoot/Status", "FLYWHEEL SPINNING...");
    }

    @Override
    public void execute() {
        // Live RPS — change on Shuffleboard, applies instantly
        //targetRPS = SmartDashboard.getNumber("AutoShoot/ManualRPS", 0.0);
        double distance = SwerveSubsystem.botPose.getTranslation().getDistance(Constants.FieldPoses.Hub);
        shooter.setVelocity(ShooterConstants.SHOOTER_MAP.get(distance));

        // Column (shooter4 / CAN 24) starts after delay
        
        boolean ready = shooter.isAtSpeed(targetRPS);

        if (ready) {
            shooter.column();
        } else {
            shooter.stop1();

        }

      
        SmartDashboard.putNumber ("AutoShoot/TargetRPS",    targetRPS);
        SmartDashboard.putBoolean("AutoShoot/ShooterReady", ready);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();  // stops shooter1/2/3
        shooter.stop1(); // stops shooter4 (column)
        SmartDashboard.putString ("AutoShoot/Status",       "STOPPED");
        SmartDashboard.putBoolean("AutoShoot/ShooterReady", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}