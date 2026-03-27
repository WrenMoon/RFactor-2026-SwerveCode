package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;


import java.io.File;

import javax.print.attribute.standard.PageRanges;

// import frc.robot.Commands.*;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Subsystems.*;

public class RobotContainer {
  
  //Creating all the subsystems
  private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "JsonConstants"));
  final CommandPS5Controller WakakeController = new CommandPS5Controller(0);
  final CommandPS5Controller AmaryanController = new CommandPS5Controller(1);


  public RobotContainer() {

    configureBindings();

     //Default Swerve Command to drive with 3 axis on PS5 Controller
     Command driveSwerve = swerve.driveCommand(
      () -> MathUtil.applyDeadband((-WakakeController.getLeftY() * (((WakakeController.getR2Axis()+ 1)/2) + 3)/4) * Math.max(1 - ((WakakeController.getL2Axis()+ 1)/2), 0.3), Constants.ControllerDeadband),
      () -> MathUtil.applyDeadband((-WakakeController.getLeftX() * (((WakakeController.getR2Axis()+ 1)/2) + 3)/4) * Math.max(1 - ((WakakeController.getL2Axis()+ 1)/2), 0.3), Constants.ControllerDeadband),
      () -> MathUtil.applyDeadband((-WakakeController.getRightX()) * Math.max(1 - ((WakakeController.getL2Axis()+ 1)/2), 0.3), Constants.ControllerDeadband), false, true); //Control heading with right joystick

    // Command driveSwerve = swerve.driveCommand(
    //   () -> MathUtil.applyDeadband(-WakakeController.getLeftY(), Constants.ControllerDeadband),
    //   () -> MathUtil.applyDeadband(-WakakeController.getLeftX(), Constants.ControllerDeadband),
    //   () -> (WakakeController.L1().getAsBoolean()? 1 : 0) - (WakakeController.R1().getAsBoolean()? 1 : 0), false, true);


    // if(Constants.blueAlliance){

    // // Main method of driving the swerve, using heading control for blue alliance
    // Command driveSwerve = swerve.driveCommand(
    //   () -> MathUtil.applyDeadband((-WakakeController.getLeftY() * (((WakakeController.getR2Axis()+ 1)/2) + 3)/4) * Math.max(1 - ((WakakeController.getL2Axis()+ 1)/2), 0.3), Constants.ControllerDeadband),
    //   () -> MathUtil.applyDeadband((-WakakeController.getLeftX() * (((WakakeController.getR2Axis()+ 1)/2) + 3)/4) * Math.max(1 - ((WakakeController.getL2Axis()+ 1)/2), 0.3), Constants.ControllerDeadband),
    //   () -> getHeadingAngleX(),
    //   () -> getHeadingAngleY()
    // );
    // swerve.setDefaultCommand(driveSwerve);

    // } else {

    // // Main method of driving the swerve, using heading control for red alliance
    // Command driveSwerve = swerve.driveCommand(
    //   () -> -1* MathUtil.applyDeadband((-WakakeController.getLeftY() * (((WakakeController.getR2Axis()+ 1)/2) + 3)/4) * Math.max(1 - ((WakakeController.getL2Axis()+ 1)/2), 0.3), Constants.ControllerDeadband),
    //   () -> -1* MathUtil.applyDeadband((-WakakeController.getLeftX() * (((WakakeController.getR2Axis()+ 1)/2) + 3)/4) * Math.max(1 - ((WakakeController.getL2Axis()+ 1)/2), 0.3), Constants.ControllerDeadband),
    //   () -> -1* getHeadingAngleX(),
    //   () -> -1* getHeadingAngleY()
    // );
    swerve.setDefaultCommand(driveSwerve);
    
  }

  /**
   * Configuring all the button bindings for all the controllers
   */
  private void configureBindings() {

    WakakeController.button(10).onTrue(Commands.runOnce(swerve::zeroGyro));

    WakakeController.touchpad().whileTrue(swerve.driveCommand(() -> 0,() -> 0, () -> 0,false, false));
    WakakeController.povUp().whileTrue(swerve.driveCommand(() -> 0.1,() -> 0, () -> 0,false, false));
    WakakeController.povDown().whileTrue(swerve.driveCommand(() -> -0.1,() -> 0, () -> 0,false, false));
    WakakeController.povLeft().whileTrue(swerve.driveCommand(() -> 0,() -> 0.1, () -> 0,false, false));
    WakakeController.povRight().whileTrue(swerve.driveCommand(() -> 0,() -> -0.1, () -> 0,false, false));
    WakakeController.button(20).whileTrue(swerve.driveCommand(() -> 0,() -> 0, () -> 1,false, false));
    WakakeController.button(20).whileTrue(swerve.driveCommand(() -> 0,() -> 0, () -> -1,false, false));
  }


  /**%
   * Command for the robot to run during autonomous
   * @return Autonomous Command of the robot for the command scheduler
   */
  public Command getAutonomousCommand() {
    return swerve.getAutonomousCommand("MLA");
  }

  public double getHeadingAngleX(){
    
    double headingX = SmartDashboard.getNumber("Swerve Target HeadingX", 0);

    if(WakakeController.triangle().getAsBoolean()){
      headingX = 0;
    } else if (WakakeController.square().getAsBoolean()){
      headingX = -1;
    } else if (WakakeController.circle().getAsBoolean()){
      headingX = 1;
    } else if (WakakeController.cross().getAsBoolean()){
      headingX = 0;
    } else if (WakakeController.R3().getAsBoolean()){

      headingX = -WakakeController.getRightX();

    } else if (Math.abs(WakakeController.getRightX()) > 0.7 || Math.abs(WakakeController.getRightY()) > 0.7){
      
      double heading = JoystickHeading();

      if (Math.abs(heading - 0) < 30){

        headingX = 0;

      } else if  (Math.abs(heading - 60) < 30){

        headingX = -1;

      } else if  (Math.abs(heading - 120) < 30){

        headingX = -1;

      } else if  (Math.abs(heading - 180) < 30){

        headingX = 0;

      } else if  (Math.abs(heading - 240) < 30){

        headingX = 1;

      } else if  (Math.abs(heading - 300) < 30){

        headingX = 1;

      } else if  (Math.abs(heading - 360) < 30){

        headingX = 0;

      }
    }

    SmartDashboard.putNumber("Swerve Target HeadingX", headingX);
    
    return headingX;
  }

  public double getHeadingAngleY(){
    
    double headingY = SmartDashboard.getNumber("Swerve Target HeadingY", 0);
    if(WakakeController.triangle().getAsBoolean()){
      headingY = 1;
    } else if (WakakeController.square().getAsBoolean()){
      headingY= 0.7;
    } else if (WakakeController.circle().getAsBoolean()){
      headingY = 0.7;
    } else if (WakakeController.cross().getAsBoolean()){
      headingY = -1;

    } else if (WakakeController.R3().getAsBoolean()){

      headingY = -WakakeController.getRightY();

    } else if (Math.abs(WakakeController.getRightX()) > 0.7 || Math.abs(WakakeController.getRightY()) > 0.7){

      double heading = JoystickHeading();

      if (Math.abs(heading - 0) <= 30){

        headingY = 1;

      } else if  (Math.abs(heading - 60) <= 30){

        headingY = 0.5773502691896258;

      } else if  (Math.abs(heading - 120) <= 30){

        headingY = -0.5773502691896258;

      } else if  (Math.abs(heading - 180) <= 30){

        headingY = -1;

      } else if  (Math.abs(heading - 240) <= 30){

        headingY = -0.5773502691896258;

      } else if  (Math.abs(heading - 300) <= 30){

        headingY = 0.5773502691896258;

      } else if  (Math.abs(heading - 360) <= 30){

        headingY = 1;

      }
    }

    SmartDashboard.putNumber("Swerve Target HeadingY", headingY);

    return headingY;
  }

  public double JoystickHeading(){
      double X = -WakakeController.getRightX();  // X component (leftward, so -X)
      double Y = -WakakeController.getRightY();  // Y component (upward)

      double resultX = -X;
      double resultY = Y;

      double angleRad = Math.atan2(resultX, resultY);

      double angleDeg = Math.toDegrees(angleRad);

      if (angleDeg < 0) {
          angleDeg += 360;
      }

      return angleDeg;
  }
} 