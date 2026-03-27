package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;


public class gridSnap extends Command {
    private final SwerveSubsystem swerve;
    private Double translationX;
    private Double translationY;
    boolean endLoop = false;

    public gridSnap(SwerveSubsystem swerve, double translationX, double translationY) {
        this.swerve = swerve;
        this.translationX = translationX;
        this.translationY = translationY;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        boolean endLoop = false;
    }

    @Override
    public void execute() {
        double Heading = swerve.getHeading().getDegrees();

        double N = Math.round((Heading - 45.0) / 90.0);
        double setpoint = 45.0 + 90.0 * N;

        // swerve.driveHeading(translationX, translationY,0, 0);
        SmartDashboard.putNumber("gridSnap/Heading", Heading);
        SmartDashboard.putNumber("gridSnap/setpoint", setpoint);
        SmartDashboard.putNumber("gridSnap/cos", Math.cos(Math.toRadians(setpoint)));
        SmartDashboard.putNumber("gridSnap/sin", Math.sin(Math.toRadians(setpoint)));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return endLoop;
    }
}
