package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FeederConstants;


public class FeederSubsystem extends SubsystemBase {

    private final TalonFX feederMotor;

    /**
     * The subsytem for the feeder mechanism of the robot
     */
    public FeederSubsystem() {
        feederMotor = new TalonFX(FeederConstants.FEEDER_ID);
        feederMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Runs the feeder forward at its fixed speed
     */
    public void runFeeder() {
        feederMotor.set(FeederConstants.FEEDER_SPEED);
    }

    /**
     * Runs the feeder backwards at its fixed speed
     */
    public void reverseFeeder() {
        feederMotor.set(FeederConstants.FEEDER_REVERSE_SPEED);
    }

    /* Stops the feeder */
    public void stopFeeder() {
        feederMotor.set(0);
    }


    @Override
    public void periodic() {
        
        // Smartdashboard for debugging
        if(Constants.smartEnable){
            SmartDashboard.putNumber("FeederSpeed", feederMotor.getVelocity().getValueAsDouble());
        }
    }
}
