package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

/**
 * Intake subsystem:
 *   CAN 25 — Roller motor  : spins to intake or outtake the game piece (duty cycle)
 *   CAN 27 — Pivot motor   : rotates the arm in/out using PID position control
 */
public class PivotSubsystem extends SubsystemBase {

    // ─── Hardware ─────────────────────────────────────────────────────────────
    private final TalonFX pivotMotor;

    public PivotSubsystem() {
        pivotMotor  = new TalonFX(IntakeConstants.INTAKE_PIVOT_ID);

        // Seed pivot position to 0 on startup (assumes arm starts retracted)
        pivotMotor.setPosition(0);

        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    }

   
    public void setPivotMotor(double speed) {
        pivotMotor.set(speed);
    }

    public Command setPivotMotor(DoubleSupplier speed){
        return run(() -> {
            // pivotMotor.set(speed.getAsDouble() + Constants.IntakeConstants.PIVOT_kG * Math.sin(getPivotAngle()));    
            pivotMotor.set(speed.getAsDouble());    
        });
    }

    public void setPivotMotorFeedForward(double speed){
        pivotMotor.set(speed + Constants.IntakeConstants.PIVOT_kG * Math.sin(getPivotAngle()));
    }

    public double getRawPivotPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    public double getPivotAngle() {
        return -(getRawPivotPosition() * Constants.IntakeConstants.PIVOT_GEAR_RATIO) - 30;
    }

    @Override
    public void periodic() {
        if (Constants.smartEnable){
            SmartDashboard.putNumber("Intake/RawPivotPosition", getRawPivotPosition());
            SmartDashboard.putNumber("Intake/PivotAngle", getPivotAngle());
            SmartDashboard.putNumber("Intake/PivotSpeed", pivotMotor.getVelocity().getValueAsDouble());
        }
    }
}