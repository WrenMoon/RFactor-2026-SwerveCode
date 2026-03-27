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
import frc.robot.RobotContainer;


public class alignPose extends Command {
    private final SwerveSubsystem swerve;
    private final PIDController xPID;
    private final PIDController yPID;
    private Double HeadingX;
    private Double HeadingY;
    private Pose2d targetPose;
    boolean endLoop = false;

    public alignPose(SwerveSubsystem swerve, Pose2d targetPose, double HeadingX, double HeadingY) {
        this.swerve = swerve;
        this.targetPose = targetPose;
        this.HeadingX = HeadingX;
        this.HeadingY = HeadingY;

        xPID = new PIDController(Constants.CV.kp,0,Constants.CV.kd);
        yPID = new PIDController(Constants.CV.kp,0,Constants.CV.kd);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        xPID.setSetpoint(targetPose.getX());
        yPID.setSetpoint(targetPose.getY());
        xPID.setTolerance(0.007);
        yPID.setTolerance(0.007);
        boolean endLoop = false;
    }

    @Override
    public void execute() {

        double translationX = xPID.calculate(swerve.getPose().getX());
        double translationY = yPID.calculate(swerve.getPose().getY());

        if(xPID.getError() < 0.1){
            xPID.setPID(Constants.CV.secondaryKp,0,Constants.CV.secondaryKd);
        }

        if(yPID.getError() < 0.1){
            yPID.setPID(Constants.CV.secondaryKp,0,Constants.CV.secondaryKd);
        }



        if(translationX > Constants.CV.MaxSpeed){
            translationX = Constants.CV.MaxSpeed;
        } else if (translationX < -Constants.CV.MaxSpeed){
            translationX = -Constants.CV.MaxSpeed;
        }

        if(translationY > Constants.CV.MaxSpeed){
            translationY = Constants.CV.MaxSpeed;
        } else if (translationY < -Constants.CV.MaxSpeed){
            translationY = -Constants.CV.MaxSpeed;
        }


        if (Constants.smartEnable){
            SmartDashboard.putNumber("Pose Align/TranslationX", translationX);
            SmartDashboard.putNumber("Pose Align/TranslationY", translationY);
            SmartDashboard.putNumber("Pose Align/Error X", xPID.getError());
            SmartDashboard.putNumber("Pose Align/Error Y", yPID.getError());
        }

        swerve.driveHeading(translationX, translationY,0, 0);

        if((xPID.atSetpoint() && yPID.atSetpoint())){
            endLoop = true;
            SmartDashboard.putBoolean("Pose Align/Aligned", true);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return endLoop;
    }
}
