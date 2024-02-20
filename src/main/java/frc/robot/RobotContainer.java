// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.auto.FireAtSpeakerCmd;
import frc.robot.commands.auto.IntakeFromGroundCmd;
import frc.robot.commands.auto.MoveToPosCmd;
import frc.robot.commands.functions.EmergencyStopMechanismsCmd;
import frc.robot.commands.functions.ShootAmpCmd;
import frc.robot.commands.functions.ShootCmd;
import frc.robot.commands.functions.SourceIntakeCmd;
import frc.robot.commands.functions.ToggleArticulateCmd;
import frc.robot.commands.functions.ToggleClimberCmd;
import frc.robot.commands.functions.ToggleFlapCmd;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;


public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  private final Joystick driverJoystick = new Joystick(OIConstants.kOperatorControllerPort);
  private final Joystick translateStick = new Joystick(OIConstants.kDriverTranslateStickPort);
  private final Joystick rotateStick = new Joystick(OIConstants.kDriverRotateStickPort);

  public RobotContainer() {

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -translateStick.getRawAxis(OIConstants.kDriverYAxis),
      () -> -translateStick.getRawAxis(OIConstants.kDriverXAxis),
      () -> rotateStick.getRawAxis(0),
      () -> !driverJoystick.getRawButton(OIConstants.kDriverCoordinateButtonId),
      () -> translateStick.getRawButton(1),
      () -> rotateStick.getRawButton(1)));

    // swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
    //             swerveSubsystem,
    //             () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
    //             () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
    //             () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
    //             () -> !driverJoystick.getRawButton(OIConstants.kDriverCoordinateButtonId),
    //             () -> driverJoystick.getRawButton(OIConstants.kDriverShootButtonId),
    //             () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonId)));


                
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driverJoystick, OIConstants.kDriverResetGyroButtonId).onTrue(swerveSubsystem.zeroHeading());
    new JoystickButton(driverJoystick, OIConstants.kDriverCoordinateButtonId).onTrue(swerveSubsystem.coordinate());
    new JoystickButton(driverJoystick, OIConstants.kDriverStopButtonId).onTrue(new EmergencyStopMechanismsCmd(shooterSubsystem, intakeSubsystem, climberSubsystem));
    new JoystickButton(driverJoystick, OIConstants.kDriverToggleClimberButtonId).onTrue(new ToggleClimberCmd(climberSubsystem));
    new JoystickButton(driverJoystick, OIConstants.kDriverSourceIntakeButtonId).whileTrue(new SourceIntakeCmd(shooterSubsystem, intakeSubsystem));

    // TRIGGERS
    new JoystickButton(driverJoystick, OIConstants.kDriverToggleFlapButtonId).whileTrue(new ToggleFlapCmd(shooterSubsystem));
    new JoystickButton(driverJoystick, OIConstants.kDriverToggleGroundIntakeButtonId).whileTrue(new ToggleArticulateCmd(intakeSubsystem));
    new JoystickButton(driverJoystick, OIConstants.kDriverRunAmpButtonId).whileTrue(new ShootAmpCmd(shooterSubsystem, intakeSubsystem));
    new JoystickButton(driverJoystick, OIConstants.kDriverRunShooterButtonId).whileTrue(new ShootCmd(shooterSubsystem, intakeSubsystem));
  }



  public Command getAutonomousCommand() {
    Pose2d[] path1 = {
      new Pose2d(1.5,7.1,new Rotation2d(0 * Math.PI / 180)),
    };

    return new SequentialCommandGroup(
          new InstantCommand(() -> SmartDashboard.putBoolean("Done Auto", false)),
          // new MoveToPosCmd(swerveSubsystem, path1, true, true), //90 * Math.PI / 180
          new FireAtSpeakerCmd(swerveSubsystem, shooterSubsystem, intakeSubsystem),
          new MoveToPosCmd(swerveSubsystem, path1, true, false), //90 * Math.PI / 180
          new IntakeFromGroundCmd(swerveSubsystem, intakeSubsystem, new Translation2d(2.85, 5.48)),
          new FireAtSpeakerCmd(swerveSubsystem, shooterSubsystem, intakeSubsystem),
          new InstantCommand(() -> SmartDashboard.putBoolean("Done Auto", true)),
          new InstantCommand(() -> swerveSubsystem.stopModules()));
  }
}
