package frc.robot.Commands;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.FeederSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;

public class LUTAutoShootCommand extends Command {

    private final ShooterSubsystem   shooter;
    private final FeederSubsystem   feeder;
    private double targetRPS;

    /**
     * A shoot command that uses Lookup table to calculate the required shooter power to reach the target and then runs the shooter
     * @param shooter
     */
    public LUTAutoShootCommand( ShooterSubsystem shooter, FeederSubsystem feeder)
    {
        this.shooter   = shooter;
        this.feeder = feeder;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {

        shooter.stop1(); // make sure column is off at start
        
        if (Constants.smartEnable){
            SmartDashboard.putString("AutoShoot/Status", "FLYWHEEL SPINNING...");
        }
    }

    @Override
    public void execute() {

        double distance = SwerveSubsystem.botPose.getTranslation().getDistance(Constants.FieldPoses.Hub);
        targetRPS = ShooterConstants.SHOOTER_MAP.get(distance);
        shooter.setVelocity(targetRPS);
        
        boolean ready = shooter.isAtSpeed(targetRPS);

        if (ready) {
            shooter.column();
            feeder.runFeeder();
        } else {
            shooter.stop1();
            feeder.stopFeeder();
        }

        // Smartdashboard for debugging
        if (Constants.smartEnable){
            SmartDashboard.putNumber ("AutoShoot/TargetRPS",    targetRPS);
            SmartDashboard.putBoolean("AutoShoot/ShooterReady", ready);
            SmartDashboard.putNumber("AutoShoot/Distance", distance);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();  
        shooter.stop1();

        // Smartdashboard for debugging
        if (Constants.smartEnable){
            SmartDashboard.putString ("AutoShoot/Status",       "STOPPED");
            SmartDashboard.putBoolean("AutoShoot/ShooterReady", false);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}