package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

import org.dyn4j.world.listener.StepListener;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;



public class gridSnap extends Command {
    private final SwerveSubsystem swerve;
    private Double translationX;
    private Double translationY;
    private Double Heading;
    private Double setpoint;
    private CommandPS5Controller driveController;
    boolean endLoop = false;;

    public gridSnap(SwerveSubsystem swerve, CommandPS5Controller driveController) {
        this.swerve = swerve;
        this.driveController = driveController;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        boolean endLoop = false;
        Heading = swerve.getHeading().getDegrees();
    }

    @Override
    public void execute() {

        double N = Math.round((Heading - 45.0) / 90.0);
        double setpoint = (45.0 + 90.0 * N);

        if(driveController.povUp().getAsBoolean()){
            translationX = Constants.povSpeed;
            translationY = 0.0;
        } else if (driveController.povDown().getAsBoolean()){
            translationX = -Constants.povSpeed;
            translationY = 0.0;
        } else if (driveController.povLeft().getAsBoolean()){
            translationX = 0.0;
            translationY = Constants.povSpeed;
        } else if (driveController.povDown().getAsBoolean()){
            translationX = 0.0;
            translationY = -Constants.povSpeed;
        } else {
            translationX = 0.0;
            translationY = 0.0;
        }

        if (Math.abs(setpoint - swerve.getHeading().getDegrees()) < 2){
            endLoop = true;
        } else{
            endLoop = false;
        }

        swerve.driveHeading(translationX, translationY, Math.sin(Math.toRadians(setpoint)), Math.cos(Math.toRadians(setpoint)));
        SmartDashboard.putNumber("gridSnap/Heading", swerve.getHeading().getDegrees());
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
