package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.io.File;

import javax.print.attribute.standard.PageRanges;

// import frc.robot.Commands.*;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Commands.LUTAutoShootCommand;
import frc.robot.Commands.alignPose;
import frc.robot.Commands.gridSnap;
import frc.robot.Commands.pivotPosCmd;
import frc.robot.Subsystems.*;

public class RobotContainer {
  
  //Creating all the subsystems
  private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "JsonConstants"));
  private final IntakeSubsystem  intakeSubsystem  = new IntakeSubsystem();
  private final PivotSubsystem pivotSubsystem  = new PivotSubsystem();
  private final FeederSubsystem  feederSubsystem  = new FeederSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  
  final CommandPS5Controller driverController = new CommandPS5Controller(0);
  final CommandPS5Controller operatorController = new CommandPS5Controller(1);


  public RobotContainer() {
 
    configureBindings();

     //Default Swerve Command to drive with 3 axis on PS5 Controller
     Command driveSwerve = swerve.driveCommand(
      () -> MathUtil.applyDeadband((-driverController.getLeftY() * (((driverController.getR2Axis()+ 1)/2) + 3)/4) * Math.max(1 - ((driverController.getL2Axis()+ 1)/2), 0.3), Constants.ControllerDeadband),
      () -> MathUtil.applyDeadband((-driverController.getLeftX() * (((driverController.getR2Axis()+ 1)/2) + 3)/4) * Math.max(1 - ((driverController.getL2Axis()+ 1)/2), 0.3), Constants.ControllerDeadband),
      () -> MathUtil.applyDeadband((-driverController.getRightX()), Constants.ControllerDeadband), false, true); //Control heading with right joystick
        // Command driveSwerve = swerve.driveCommand(
        //   () -> MathUtil.applyDeadband((-driverController.getLeftY() * (((driverController.getR2Axis()+ 1)/2) + 3)/4) * Math.max(1 - ((driverController.getL2Axis()+ 1)/2), 0.3), Constants.ControllerDeadband),
        //   () -> MathUtil.applyDeadband((-driverController.getLeftX() * (((driverController.getR2Axis()+ 1)/2) + 3)/4) * Math.max(1 - ((driverController.getL2Axis()+ 1)/2), 0.3), Constants.ControllerDeadband),
        //   () -> MathUtil.applyDeadband((-driverController.getRightX()), Constants.ControllerDeadband),
        //   () -> MathUtil.applyDeadband((-driverController.getRightY()), Constants.ControllerDeadband)); //Control heading with right joystick
    

    swerve.setDefaultCommand(driveSwerve);
  }

  /**
   * Configuring all the button bindings for all the controllers
   */
  private void configureBindings() {

    driverController.button(10).onTrue(Commands.runOnce(swerve::zeroGyro));

    driverController.touchpad().whileTrue(swerve.driveCommand(() -> 0,() -> 0, () -> 0,false, true));
    driverController.povUp().whileTrue(swerve.driveCommand(() -> Constants.povSpeed,() -> 0, () -> 0,false, true));
    driverController.povDown().whileTrue(swerve.driveCommand(() -> -Constants.povSpeed,() -> 0, () -> 0,false, true));
    driverController.povLeft().whileTrue(swerve.driveCommand(() -> 0,() -> Constants.povSpeed, () -> 0,false, true));
    driverController.povRight().whileTrue(swerve.driveCommand(() -> 0,() -> -Constants.povSpeed, () -> 0,false, false));
    driverController.L1().onTrue(new gridSnap(swerve, driverController));
    

    // ── Operator — Square: intake outtake ─────────────────────────────────
        operatorController.L1().whileTrue(
            new StartEndCommand(
                intakeSubsystem::runRollerOuttake,
                intakeSubsystem::stopRoller,
                intakeSubsystem));

        // ── Operator — Cross: intake in ───────────────────────────────────────
        operatorController.L2().whileTrue(new ParallelCommandGroup(
            new StartEndCommand(
                intakeSubsystem::runRollerIntake,
                intakeSubsystem::stopRoller,
                intakeSubsystem), new pivotPosCmd(pivotSubsystem, 90, false)));
        
        // ── Operator — R1: feeder (fire) ──────────────────────────────────────
        operatorController.R1().whileTrue(
            new StartEndCommand(
                feederSubsystem::runFeeder,
                feederSubsystem::stopFeeder,
                feederSubsystem));

        // ── Operator — L1: feeder reverse (unjam) ────────────────────────────
        operatorController.triangle().whileTrue(
            new StartEndCommand(
                feederSubsystem::reverseFeeder,
                feederSubsystem::stopFeeder,
                feederSubsystem));
              

        // ── Operator — R1: reverse shooter (unjam) ───────────────────────────
        operatorController.R1().onTrue(
            new RunCommand(shooterSubsystem::reverse, shooterSubsystem)
        ).onFalse(
            new InstantCommand(shooterSubsystem::stop, shooterSubsystem)
        );

        operatorController.povUp().whileTrue(new pivotPosCmd(pivotSubsystem, -20, true));
        operatorController.povDown().whileTrue(new pivotPosCmd(pivotSubsystem, 90, true));

        // driverController.cross().whileTrue((new SequentialCommandGroup(new alignPose(swerve, swerve.getPose(), Constants.FieldPoses.Hub.getX() - swerve.getPose().getX(), Constants.FieldPoses.Hub.getY() - swerve.getPose().getY()), new LUTAutoShootCommand(shooterSubsystem))));

        // operatorController.circle().onTrue(
        //     new RunCommand(shooterSubsystem::column, shooterSubsystem)
        // ).onFalse(
        //     new InstantCommand(shooterSubsystem::stop1, shooterSubsystem)
        // );

        // operatorController.R3().whileTrue(
        // new ArcShootCommand(limelightSubsystem, driveSubsystem,
        //                   shooterSubsystem, operatorController));
        // operatorController.R2().whileTrue(
        // new LUTAutoShootCommand(shooterSubsystem));

  }


  /**%
   * Command for the robot to run during autonomous
   * @return Autonomous Command of the robot for the command scheduler
   */
  public Command getAutonomousCommand() {
    return swerve.getAutonomousCommand("autoname");
  }
} 