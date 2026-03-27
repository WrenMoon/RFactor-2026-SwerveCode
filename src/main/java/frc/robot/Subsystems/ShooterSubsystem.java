package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX shooter1;
    private final TalonFX shooter2;
    private final TalonFX shooter3;
    private final TalonFX shooter4;

    public ShooterSubsystem() {
        shooter1 = new TalonFX(ShooterConstants.SHOOTER_1_ID);
        shooter2 = new TalonFX(ShooterConstants.SHOOTER_2_ID);
        shooter3 = new TalonFX(ShooterConstants.SHOOTER_3_ID);
        shooter4 = new TalonFX(ShooterConstants.SHOOTER_4_ID);

        shooter1.setNeutralMode(NeutralModeValue.Coast);
        shooter2.setNeutralMode(NeutralModeValue.Coast);
        shooter3.setNeutralMode(NeutralModeValue.Coast);
        shooter4.setNeutralMode(NeutralModeValue.Coast);
    }

    // ─── API ──────────────────────────────────────────────────────────────────

    /** Spin up to fixed shoot velocity from Constants (manual shoot). */
    public void spinUp() {
        setVelocity(ShooterConstants.SHOOT_VELOCITY_RPS);
    }

    /**
     * Spin up to a DYNAMIC velocity — used by AutoShootCommand.
     * All 3 motors run at the same RPS.
     */
    public void setVelocity(double rps) {
        shooter1.set(rps);
        shooter2.set(-rps);
        shooter3.set(rps);
        // shooter4.setControl(velocityReq.withVelocity(rps));
    }
    public void column(){
        shooter4.set(-0.8);
    }

    /** Reverse all motors (unjam). */
    public void reverse() {
        double target = ShooterConstants.REVERSE_VELOCITY_RPS;
        shooter1.set(target);
        shooter2.set(-target);
        shooter3.set(target);
        shooter4.set(-target);
    }

    /** Stop all motors. */
    public void stop() {
        shooter1.set(0);
        shooter2.set(0);
        shooter3.set(0);
        shooter4.set(0);
    }
    public void stop1(){
        shooter4.set(0);
    }

    /**
     * Check if all motors are within tolerance of a FIXED target (Constants).
     * Used by manual spinUp() check.
     */
    public boolean isAtSpeed() {
        return isAtSpeed(ShooterConstants.SHOOT_VELOCITY_RPS);
    }

    /**
     * Check if all motors are within tolerance of a DYNAMIC target.
     * Used by AutoShootCommand.
     */
    public boolean isAtSpeed(double targetRPS) {
        double tol = ShooterConstants.VELOCITY_TOLERANCE_RPS;
        double v1  = Math.abs(shooter1.getVelocity().getValue().in(Units.RotationsPerSecond));
        double v2  = Math.abs(shooter2.getVelocity().getValue().in(Units.RotationsPerSecond));
        double v3  = Math.abs(shooter3.getVelocity().getValue().in(Units.RotationsPerSecond));
        return (Math.abs(v1 - targetRPS) < tol);
        // return Math.abs(v1 - targetRPS) < tol
        //     && Math.abs(v2 - targetRPS) < tol
        //     && Math.abs(v3 - targetRPS) < tol;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Shooter/AtSpeed",      isAtSpeed());
        SmartDashboard.putNumber ("Shooter/Velocity1",    shooter1.getVelocity().getValue().in(Units.RotationsPerSecond));
        SmartDashboard.putNumber ("Shooter/Velocity2",    shooter2.getVelocity().getValue().in(Units.RotationsPerSecond));
        SmartDashboard.putNumber ("Shooter/Velocity3",    shooter3.getVelocity().getValue().in(Units.RotationsPerSecond));
    }
}