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


import java.io.File;

import javax.print.attribute.standard.PageRanges;

// import frc.robot.Commands.*;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Commands.gridSnap;
import frc.robot.Subsystems.*;

public class RobotContainer {
  
  //Creating all the subsystems
  private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "JsonConstants"));
  private final IntakeSubsystem  intakeSubsystem  = new IntakeSubsystem();
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

    swerve.setDefaultCommand(driveSwerve);
  }

  /**
   * Configuring all the button bindings for all the controllers
   */
  private void configureBindings() {

    driverController.button(10).onTrue(Commands.runOnce(swerve::zeroGyro));

    driverController.touchpad().whileTrue(swerve.driveCommand(() -> 0,() -> 0, () -> 0,false, false));
    driverController.povUp().whileTrue(swerve.driveCommand(() -> 0.1,() -> 0, () -> 0,false, false));
    driverController.povDown().whileTrue(swerve.driveCommand(() -> -0.1,() -> 0, () -> 0,false, false));
    driverController.povLeft().whileTrue(swerve.driveCommand(() -> 0,() -> 0.1, () -> 0,false, false));
    driverController.povRight().whileTrue(swerve.driveCommand(() -> 0,() -> -0.1, () -> 0,false, false));
    driverController.button(1).whileTrue(new gridSnap(swerve, 0, 0));
    

    // ── Operator — Intake ─────────────────────────────────────────────────
    // R2 held -> deploy arm + rollers IN
    operatorController.R2().whileTrue(
        new StartEndCommand(
            () -> { intakeSubsystem.deploy(); intakeSubsystem.runRollerIntake(); },
            ()  -> intakeSubsystem.stopRoller(),
            intakeSubsystem));


    // L2 held -> deploy arm + rollers OUT (eject)
    operatorController.L2().whileTrue(
        new StartEndCommand(
            () -> { intakeSubsystem.deploy(); intakeSubsystem.runRollerOuttake(); },
            ()  -> intakeSubsystem.stopRoller(),
            intakeSubsystem));

    // Circle -> retract / stow arm
    operatorController.circle().onTrue(
        new InstantCommand(intakeSubsystem::retract, intakeSubsystem));

    // ── Operator — Feeder ─────────────────────────────────────────────────
    operatorController.R1().whileTrue(
        new StartEndCommand(
            feederSubsystem::runFeeder,
            feederSubsystem::stopFeeder,
            feederSubsystem));

    operatorController.L1().whileTrue(
        new StartEndCommand(
            feederSubsystem::reverseFeeder,
            feederSubsystem::stopFeeder,
            feederSubsystem));

    // ── Operator — Shooter ────────────────────────────────────────────────
    operatorController.triangle().whileTrue(
        new StartEndCommand(
            shooterSubsystem::spinUp,
            shooterSubsystem::stop,
            shooterSubsystem));

    operatorController.square().whileTrue(
        new StartEndCommand(
            shooterSubsystem::reverse,
            shooterSubsystem::stop,
            shooterSubsystem));
  
  }


  /**%
   * Command for the robot to run during autonomous
   * @return Autonomous Command of the robot for the command scheduler
   */
  public Command getAutonomousCommand() {
    return swerve.getAutonomousCommand("autoname");
  }
} 