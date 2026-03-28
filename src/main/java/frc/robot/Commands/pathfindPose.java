package frc.robot.Commands;
import frc.robot.Constants;
import frc.robot.Subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class pathfindPose extends Command {
    private final SwerveSubsystem swerve;
    private DoubleSupplier HeadingX;
    private DoubleSupplier HeadingY;
    private Pose2d targetPose;

    /**
     * A command to pathfind to a given target pose, avoiding obstacles along the way.
     * 
     * @param swerve the swerve subsytem to drive
     * @param targetPose the target position to reach
     * @param HeadingX the horizontal component of the desired Heading
     * @param HeadingY the vertical component of the desired Heading
     */
    public pathfindPose(SwerveSubsystem swerve, Pose2d targetPose, DoubleSupplier HeadingX,
            DoubleSupplier HeadingY) {
        this.swerve = swerve;
        this.HeadingX = HeadingX;
        this.HeadingY = HeadingY;
        this.targetPose = targetPose;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(swerve.driveToPose(() -> targetPose, 0), new alignPose(swerve, targetPose, HeadingX.getAsDouble(), HeadingY.getAsDouble())));
        
        // Smartdashboard for debugging
        if (Constants.smartEnable){
            SmartDashboard.putBoolean("pathfinding", true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        
        // Smartdashboard for debugging
        if (Constants.smartEnable){
            SmartDashboard.putBoolean("pathfinding", false);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
