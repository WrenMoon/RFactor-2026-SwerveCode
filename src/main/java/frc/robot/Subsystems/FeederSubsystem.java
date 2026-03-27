package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FeederConstants;

/**
 * Feeder subsystem:
 *   CAN 26 — Feeds game piece from intake toward the shooter column.
 */
public class FeederSubsystem extends SubsystemBase {

    private final TalonFX feederMotor;

    public FeederSubsystem() {
        feederMotor = new TalonFX(FeederConstants.FEEDER_ID);
        feederMotor.setNeutralMode(NeutralModeValue.Coast);
    }


    // ─── API ──────────────────────────────────────────────────────────────────

    /** Run feeder forward to push game piece toward shooter. */
    public void runFeeder() {
        feederMotor.set(FeederConstants.FEEDER_SPEED);
    }

    /** Run feeder in reverse to unjam. */
    public void reverseFeeder() {
        feederMotor.set(FeederConstants.FEEDER_REVERSE_SPEED);
    }

    /** Stop feeder. */
    public void stopFeeder() {
        feederMotor.set(0);
    }

    // ─── Periodic ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
     if(Constants.smartEnable){
        SmartDashboard.putNumber("FeederSpeed", feederMotor.getVelocity().getValueAsDouble());
     }
    }
}
