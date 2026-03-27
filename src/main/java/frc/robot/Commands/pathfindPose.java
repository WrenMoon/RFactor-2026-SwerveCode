package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.*;


public class pathfindPose extends Command {
    private final SwerveSubsystem swerve;
    private DoubleSupplier HeadingX;
    private DoubleSupplier HeadingY;
    private Pose2d targetPose;
    private boolean poseUpdated;

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
        poseUpdated = false;
    }

    @Override
    public void execute() {
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(swerve.driveToPose(() -> targetPose, 0), new alignPose(swerve, targetPose, HeadingX.getAsDouble(), HeadingY.getAsDouble())));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
