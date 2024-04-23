package frc.robot.subsystems;

import java.util.Map;
import java.util.concurrent.TimeUnit;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;


public class SwerveSubsystem extends SubsystemBase {
    
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kModuleDriveEncoderReversed,
        DriveConstants.kModuleTurningEncoderReversed,
        DriveConstants.kFrontLeftCANCoderId,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kModuleCANCoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kModuleDriveEncoderReversed,
        DriveConstants.kModuleTurningEncoderReversed,
        DriveConstants.kFrontRightCANCoderId,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kModuleCANCoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kModuleDriveEncoderReversed,
        DriveConstants.kModuleTurningEncoderReversed,
        DriveConstants.kBackLeftCANCoderId,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kModuleCANCoderReversed);

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kModuleDriveEncoderReversed,
        DriveConstants.kModuleTurningEncoderReversed,
        DriveConstants.kBackRightCANCoderId,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kModuleCANCoderReversed);

    public final AHRS gyro = new AHRS(SPI.Port.kMXP);
    public final LimeLight limeLight = new LimeLight();
    public Pose2d pose;
    public Pose2d visionPose;
    public boolean isAllianceBlue;
    public int tick = 0;
    public double[] headingBuffer = new double[10];
    public Translation2d[] translationBuffer = new Translation2d[5];
    boolean goodHeadingBuffer = false;
    boolean goodTranslationBuffer = false;
    public boolean fieldOriented = false;

    private final GenericEntry sb_gyro, sb_voltage, sb_time, sb_coord;
    

    public SwerveModule[] swerveModules = {frontLeft, frontRight, backLeft, backRight};

    public final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
            Constants.DriveConstants.kDriveKinematics, getRotation2d(),
            new SwerveModulePosition[] {
              backLeft.getPosition(),
              backRight.getPosition(),
              frontLeft.getPosition(),
              frontRight.getPosition()
            }, new Pose2d(0, 0, new Rotation2d()));

    public SwerveSubsystem() {

        try {TimeUnit.SECONDS.sleep(1);}
        catch(InterruptedException e){}
        resetPose(new Pose2d(0, 0, new Rotation2d(0)));

        Alliance test = Alliance.Red;

        if (test == Alliance.Red) {
            this.isAllianceBlue = false;
        } else if (test == Alliance.Blue) {
            this.isAllianceBlue = true;
        }

        this.tick = 0;

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                gyro.setAngleAdjustment(180);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();



        sb_gyro = Shuffleboard.getTab("Driver")
            .add("Gyro", 0.0)
            .withWidget(BuiltInWidgets.kGyro)
            .withPosition(0, 2)
            .withSize(3, 3)
            .getEntry();

        sb_voltage = Shuffleboard.getTab("Driver")
            .add("Voltage", 0.0)
            .withWidget(BuiltInWidgets.kGraph)
            .withPosition(8, 0)
            .withSize(4, 3)
            .getEntry();

        sb_time = Shuffleboard.getTab("Driver")
            .add("Time", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0)
            .withSize(3, 1)
            .getEntry();
        
        sb_coord = Shuffleboard.getTab("Driver")
            .add("Coordinates", "")
            .withPosition(8, 3)
            .withSize(4, 1)
            .getEntry();

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        3.6, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public Command zeroHeading() {
        System.out.println("Gyro Reset");
        return Commands.runOnce(() -> gyro.reset()); // Returns a command to be used on button press
    }

    public Command coordinate() {
        System.out.println("new button worked");
        return Commands.runOnce(() -> coordinateFunction());
    }

    public double getHeading() {
        return Math.IEEEremainder(DriveConstants.kGyroReversed ? gyro.getAngle() * -1 : gyro.getAngle(), 360);
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
            new SwerveModuleState[] {
                backLeft.getState(),
                backRight.getState(),
                frontLeft.getState(),
                frontRight.getState()
            }
        );
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPosition(getRotation2d(), new SwerveModulePosition[] {
            backLeft.getPosition(),
            backRight.getPosition(),
            frontLeft.getPosition(),
            frontRight.getPosition()
          }, pose);
    }

    @Override
    public void periodic() {
        tick++;

        m_poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getRotation2d(), new SwerveModulePosition[] {
            backLeft.getPosition(),
            backRight.getPosition(),
            frontLeft.getPosition(),
            frontRight.getPosition()
          });

        visionPose = limeLight.getPose();

        if (visionPose.getX() != 0 && visionPose.getY() != 0) {
            m_poseEstimator.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
        }

        sb_gyro.setDouble(getHeading() - 180);
        sb_voltage.setDouble(RobotController.getBatteryVoltage());
        sb_time.setDouble(DriverStation.getMatchTime());
        sb_coord.setString(getPose().toString());
    }
    
    public void coordinateFunction() {
        resetPose(new Pose2d(0, 0, new Rotation2d(0)));
        backLeft.resetEncoders();
        backRight.resetEncoders();
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
    }

    public void driveRobotRelative(ChassisSpeeds speed) {
        //ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speed, 0.02); is this needed?
        SwerveModuleState states[] = DriveConstants.kDriveKinematics.toSwerveModuleStates(speed);
        setModuleStates(states);
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        backLeft.setDesiredState(desiredStates[0]);
        backRight.setDesiredState(desiredStates[1]);
        frontLeft.setDesiredState(desiredStates[2]);
        frontRight.setDesiredState(desiredStates[3]);
    }
}
