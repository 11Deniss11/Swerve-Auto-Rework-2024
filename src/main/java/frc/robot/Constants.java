package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
        public static final double kDriveMotorGearRatio = 8.14 / 1.0; // Drive ratio of 8.14 : 1
        public static final double kTurningMotorGearRatio = 1.0 / (150.0 / 7.0); // Turning ratio of (150 / 7) : 1
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;
        public static final double kPTurning = 0.5; // For PID
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(29.5); // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(29.5); // Distance between front and back wheels

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        // DRIVE Motor Ports
        public static final int kFrontLeftDriveMotorPort = 10;
        public static final int kBackLeftDriveMotorPort = 3;
        public static final int kFrontRightDriveMotorPort = 6;
        public static final int kBackRightDriveMotorPort = 8;

        // TURNING Motor Ports
        public static final int kFrontLeftTurningMotorPort = 2;
        public static final int kBackLeftTurningMotorPort = 4;
        public static final int kFrontRightTurningMotorPort = 5;
        public static final int kBackRightTurningMotorPort = 7;

        // CANCoder Ids
        public static final int kFrontLeftCANCoderId = 22;
        public static final int kBackLeftCANCoderId = 24;
        public static final int kFrontRightCANCoderId = 23;
        public static final int kBackRightCANCoderId = 21;

        // Invert booleans | We use MK4i modules so the turning motors are inverted
        public static final boolean kModuleTurningEncoderReversed = true;
        public static final boolean kModuleDriveEncoderReversed = false;
        public static final boolean kModuleCANCoderReversed = false;
        public static final boolean kGyroReversed = true;

        // Turning encoder offsets
        // public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -2.6;
        // public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 3.14159;
        // public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.436332;
        // public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 1.53589;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -148 * Math.PI / 180;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 180 * Math.PI / 180;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 25 * Math.PI / 180;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 88 * Math.PI / 180;

        // Robot speeds
        public static final double kPhysicalMaxSpeedMetersPerSecond = 3.6; // PHYSICAL max speed of the modules (safety cap) 3.6
        public static final double kTeleDriveMaxSpeedMetersPerSecond = 2.7; // Max speed set for teleop
        public static final double kTeleBoostDriveMaxSpeedMetersPerSecond = 3.2; // Max speed set for boost teleop
        public static final double kTeleSlowDriveMaxSpeedMetersPerSecond = 1.25; // Max speed set for slow teleop

        // Robot turning speeds
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 3;

        // Robot acceleration
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 4;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 4;
    }

    public static final class OIConstants {
        // Ports
        public static final int kOperatorControllerPort = 0;
        public static final int kDriverTranslateStickPort = 1;
        public static final int kDriverRotateStickPort = 2;
        public static final double kDeadband = 0.1;

        // Joysticks
        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 2;

        // Joystick Triggers
        public static final int kDriverBoostButtonId = 1;

        // Joystick Hatswitches
        public static final int kDriverSnapBackButtonId = 15;

        // Buttons
        public static final int kDriverButtonId_climberDown = 1; // X button
        public static final int kDriverButtonId_resetGyro = 2; // B button
        public static final int kDriverButtonId_toggleFlap = 3; //  button
        public static final int kDriverButtonId_climberUp = 4; // Y button
        public static final int kDriverButtonId_stop = 10; // "Start" button

        // POV
        public static final int kDriverPOVId_up = 0;
        public static final int kDriverPOVId_right = 90;
        public static final int kDriverPOVId_down = 180;
        public static final int kDriverPOVId_left = 270;

        // Triggers
        public static final int kDriverTriggerId_sourceIntake = 5; // Left bumper
        public static final int kDriverTriggerId_toggleIntake = 6; // Right bumper
        public static final int kDriverTriggerId_ampOut = 7; // Left trigger
        public static final int kDriverTriggerId_shoot = 8; // Right trigger
    } 

    public static final class ShooterConstants {
        public static final int kShooterSpinMotorId_1 = 50;
        public static final int kShooterSpinMotorId_2 = 52;
        public static final int kShooterFlapServoId_1 = 9;
        public static final int kShooterFlapServoId_2 = 8;

        public static final double kShooterFlywheelSpeed = 1.0;
        public static final double kShooterIntakeSpeed = 0.25;
        public static final double kShooterAmpSpeed = 0.15;

        public static final double kShooterFlapSpeakerPos = 65;

        public static final double kShooterFlapDefaultPos = 0;

        public static final double kShooterFlapAmpPos = 40;

        public static final double kShooterSpeedCap = 1.0;
    }

    public static final class IntakeConstants {
        public static final int kIntakeMotorId = 51;
        public static final int kIntakeArticulateMotorId = 54;

        public static final double kIntakeMotorSpeed_ground = 0.7;
        public static final double kIntakeMotorSpeed_source = 0.4;
        public static final double kIntakeMotorSpeed_in = 0.25;
        public static final double kIntakeMotorSpeed_out = -0.5;
        public static final double kIntakeMotorSpeed_amp = -0.72;

        public static final double kIntakeArticulateSpeed = 0.4;
        public static final double kIntakeArticulateAccelerationUnitsPerSecond = 3;

        public static final double kIntakeDesiredPos_amp = -20.5;
        public static final double kIntakeDesiredPos_store = 0;
        public static final double kIntakeDesiredPos_out = -44.5;
    }

    public static final class AutoConstants {

        //Speed
        public static final double kAutoMaxSpeedMetersPerSecond = 1.25; // Max speed set for auto
        public static final double kAutoGroundIntakingMaxSpeedMetersPerSecond = 1.25;

        //Accel
        public static final double kAutoMaxAccelerationUnitsPerSecond = 6;
        public static final double kAutoMaxAngularAccelerationUnitsPerSecond = 4;

        //Turning speed
        public static final double kAutoMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

        //Min Speed
        public static final double kAutoMinSpeed = 0.02;
        public static final double kAutoMinTurnSpeedRadians = 0.05;
    
        //Tick things
        public static final int kAutoSpeakerShotCheckTicks = 50;
        public static final int kAutoGroundIntakeCheckTicks = 7;

        // Tolerances
        public static final double kAutoToleranceMeters = 0.09;
        public static final double kAutoToleranceDegrees = 5.0;

        public static  final int kAutoDepositCheckTicks = 300;
        public static final int kAutoSourceColorCheckTick = 100;
        public static int kAutoCloseSpeakerShotCheckTick = 400;
    }
}