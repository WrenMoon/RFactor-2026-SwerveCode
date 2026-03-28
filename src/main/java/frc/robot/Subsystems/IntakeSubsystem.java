package frc.robot.Subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX rollerMotor;

    /*The subsystem for the intake rollers of the robot*/
    public IntakeSubsystem() {
        rollerMotor = new TalonFX(IntakeConstants.INTAKE_ROLLER_ID);
        rollerMotor.setNeutralMode(NeutralModeValue.Coast);
    }


    /** Run rollers inward to collect a game piece. */
    public void runRollerIntake() {
        rollerMotor.set(IntakeConstants.ROLLER_INTAKE_SPEED);
    }

    /** Run rollers outward to eject a game piece. */
    public void runRollerOuttake() {
        rollerMotor.set(IntakeConstants.ROLLER_OUTTAKE_SPEED);
    }

    /** Stop rollers. */
    public void stopRoller() {
        rollerMotor.set(0);
    }


    @Override
    public void periodic() {
        
        // Smartdashboard for debugging
        if (Constants.smartEnable){
            SmartDashboard.putNumber("Intake/RollerSpeed", rollerMotor.getVelocity().getValueAsDouble());
        }
    }
}