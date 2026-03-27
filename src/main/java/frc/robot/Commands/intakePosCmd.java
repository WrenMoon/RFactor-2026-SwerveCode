package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.IntakeSubsystem;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class intakePosCmd extends Command {
    private final IntakeSubsystem intake;
    private final double targetPose;
    private final boolean holdPID;
    private PIDController PIDintake;
    private boolean endLoop = false;

    /**
     * A command to move the intake to an encoder setpoint using PID Feedback and
     * Gravity compensation feedforward.
     * 
     * @param intake        the intake subsystem to move
     * @param targetPose the target pose in dergees to move to
     * @param holdPID    whether or not to hold the PID loop after acceptable error
     *                   is achieved
     */
    public intakePosCmd(IntakeSubsystem intake, double targetPose, boolean holdPID) {
        this.intake = intake;
        this.targetPose = targetPose;
        this.holdPID = holdPID;
        PIDintake = new PIDController(Constants.IntakeConstants.PIVOT_kP, 0, Constants.IntakeConstants.PIVOT_kD);

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        endLoop = false;
        PIDintake.setSetpoint(targetPose); // PID setpoint
    }

    @Override
    public void execute() {

        double speed = PIDintake.calculate(intake.getPivotAngle()); // PID Correction value
        speed = speed + Constants.IntakeConstants.PIVOT_kG * Math.cos(Math.toRadians(intake.getPivotAngle())); // Feedforward Gravity compensation

        speed = Math.min(Math.max(speed, -Constants.IntakeConstants.PivotMaxSpeed), Constants.IntakeConstants.PivotMaxSpeed); // Applying Speed Limits

        // Smartdashboard for debugging
        if (Constants.smartEnable) {
            SmartDashboard.putBoolean("intake/intakePosCmd", true);
            SmartDashboard.putNumber("intake/intake encoder", intake.getRawPivotPosition());
            SmartDashboard.putNumber("intake/intake Target Pose", targetPose);
            SmartDashboard.putNumber("intake/intake PID speed", speed);
            SmartDashboard.putNumber("intake/intake Degrees", intake.getPivotAngle());
            SmartDashboard.putBoolean("intake/intake hold", holdPID);
        }

        if (Math.abs(targetPose - intake.getPivotAngle()) < 3.5 && !holdPID) { // endcase when setpoint achieved. Only if holdPID is false
            endLoop = true;
        }

        intake.setPivotMotor(speed); // applies the speed to the motor
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPivotMotor(0); // stop the motor when the command is stopped

        // Smartdashboard for debugging
        if (Constants.smartEnable) {
            SmartDashboard.putBoolean("intake/intakePosCmd", false);
        }
    }

    @Override
    public boolean isFinished() {
        return endLoop; // ends the command when setpoint is reached and holdPID is false
    }
}
