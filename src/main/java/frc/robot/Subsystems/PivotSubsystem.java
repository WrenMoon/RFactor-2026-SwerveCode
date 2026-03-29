package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;


public class PivotSubsystem extends SubsystemBase {

    private final TalonFX pivotMotor;

    /*The subsystem for the pivot of the intake of the robot */
    public PivotSubsystem() {

        pivotMotor  = new TalonFX(IntakeConstants.INTAKE_PIVOT_ID);
        pivotMotor.setPosition(0);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    }

   /**
    * A function to set the speed of the pivot
    * @param speed the desired speed
    */
    public void setPivotMotor(double speed) {
        pivotMotor.set(speed);
    }

    /**
     * A command to set the speed of the pivot
     * @param speed the desired speed
     * @return A runnable command
     */
    public Command setPivotMotor(DoubleSupplier speed){
        return run(() -> {
            pivotMotor.set(speed.getAsDouble());    
            SmartDashboard.putNumber("AAAA", speed.getAsDouble());
        });
    }

    /**
     * 
     * @return the raw encoder value of the pivot motor
     */
    public double getRawPivotPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    /**
     * 
     * @return the adjusted pivot arm angle
     */
    public double getPivotAngle() {
        return -(getRawPivotPosition() * Constants.IntakeConstants.PIVOT_GEAR_RATIO) - 30;
    }

    @Override
    public void periodic() {


        // Smartdashboard for debugging
        if (Constants.smartEnable){
            SmartDashboard.putNumber("Intake/RawPivotPosition", getRawPivotPosition());
            SmartDashboard.putNumber("Intake/PivotAngle", getPivotAngle());
            SmartDashboard.putNumber("Intake/PivotSpeed", pivotMotor.getVelocity().getValueAsDouble());
        }
    }
}