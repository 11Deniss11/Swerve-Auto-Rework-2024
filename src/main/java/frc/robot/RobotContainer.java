// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.functions.EmergencyStopMechanismsCmd;
import frc.robot.commands.functions.ShootAmpCmd;
import frc.robot.commands.functions.ShootCmd;
import frc.robot.commands.functions.SourceIntakeCmd;
import frc.robot.commands.functions.ToggleArticulateCmd;
import frc.robot.commands.functions.ToggleClimberCmd;
import frc.robot.commands.functions.ToggleFlapCmd;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ClimberSubsystem;


public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(ledSubsystem);

  private final Joystick driverJoystick = new Joystick(OIConstants.kOperatorControllerPort);
  private final Joystick translateStick = new Joystick(OIConstants.kDriverTranslateStickPort);
  private final Joystick rotateStick = new Joystick(OIConstants.kDriverRotateStickPort);

  UsbCamera intakeCamera;

  public RobotContainer() {

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -translateStick.getRawAxis(OIConstants.kDriverYAxis),
      () -> -translateStick.getRawAxis(OIConstants.kDriverXAxis),
      () -> rotateStick.getRawAxis(0),
      () -> translateStick.getRawButton(2),
      () -> translateStick.getRawButton(1), // Trigger
      () -> rotateStick.getRawButton(1))); // Trigger

    // swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
    //             swerveSubsystem,
    //             () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
    //             () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
    //             () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
    //             () -> !driverJoystick.getRawButton(OIConstants.kDriverCoordinateButtonId),
    //             () -> driverJoystick.getRawButton(OIConstants.kDriverShootButtonId),
    //             () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonId)));

    intakeCamera = CameraServer.startAutomaticCapture(0);
                
    configureBindings();
  }

  private void configureBindings() {
    // BUTTONS
    new JoystickButton(driverJoystick, OIConstants.kDriverButtonId_resetGyro).onTrue(swerveSubsystem.zeroHeading());
    new JoystickButton(driverJoystick, OIConstants.kDriverButtonId_climberUp).onTrue(new ToggleClimberCmd(climberSubsystem, true));
    new JoystickButton(driverJoystick, OIConstants.kDriverButtonId_climberDown).onTrue(new ToggleClimberCmd(climberSubsystem, false));
    new JoystickButton(driverJoystick, OIConstants.kDriverButtonId_toggleFlap).whileTrue(new ToggleFlapCmd(shooterSubsystem));

    new JoystickButton(driverJoystick, OIConstants.kDriverButtonId_stop).onTrue(new EmergencyStopMechanismsCmd(shooterSubsystem, intakeSubsystem, climberSubsystem));


    // // POV
    // new POVButton(driverJoystick, OIConstants.kDriverPOVId_left).whileTrue(new DepositToAmpCmd(swerveSubsystem, shooterSubsystem, intakeSubsystem));
    // new POVButton(driverJoystick, OIConstants.kDriverPOVId_down).whileTrue(new SequentialCommandGroup(new SimpleFireAtSpeakerCmd(swerveSubsystem, shooterSubsystem, intakeSubsystem)));


    // TRIGGERS
    new JoystickButton(driverJoystick, OIConstants.kDriverTriggerId_sourceIntake).whileTrue(new SourceIntakeCmd(shooterSubsystem, intakeSubsystem));
    new JoystickButton(driverJoystick, OIConstants.kDriverTriggerId_toggleIntake).whileTrue(new ToggleArticulateCmd(intakeSubsystem));
    new JoystickButton(driverJoystick, OIConstants.kDriverTriggerId_ampOut).whileTrue(new ShootAmpCmd(intakeSubsystem, ledSubsystem));
    new JoystickButton(driverJoystick, OIConstants.kDriverTriggerId_shoot).whileTrue(new ShootCmd(shooterSubsystem, intakeSubsystem, ledSubsystem));
  }




  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Big Test Auto");
  }
}
// Import WaitCommand for waiting IS WHAT I WOULD SAY IF WE DIDNT HAVE PATH PLANNER YEAAAAAAAAAAAAAAAA