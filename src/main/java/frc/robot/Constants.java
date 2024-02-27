// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(21);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(25.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kFrontLeftTurningCanId = 1;

    public static final int kRearLeftDrivingCanId = 6;
    public static final int kRearLeftTurningCanId = 5;

    public static final int kFrontRightDrivingCanId = 4;
    public static final int kFrontRightTurningCanId = 3;

    public static final int kRearRightDrivingCanId = 8;
    public static final int kRearRightTurningCanId = 7;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kFreeSpeedRpm = 5676;
    public static final double kDrivingMotorFreeSpeedRps = kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kBlueWheelDiameterMeters = .08;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCopilotControllerPort = 1;
    public static final double kDeadband = 0.05;

  }

  public static final class IntakeConstants {
    public static final int IntakeMotor1Id = 9;
    // public static final int IntakeMotor2Id = 10;
    public static final double IntakeSpeed = .43;
    public static final double DefaultSpeed = 0;
  }

  public static final class ArmConstants {
    public static final int armMotorId = 20;
  }

  public static final class ClimberConstants {
    public static final int climberLeftMotorID = 10;
    public static final int climberRightMotorID = 11;
    public static final double kClimberP = .01;
    public static final double kClimberI = 0;
    public static final double kClimberD = 0;
    public static final double kClimberFF = 0;
    public static final IdleMode kClimberIdleMode = IdleMode.kBrake;
    public static final double kClimberDown = 80;
    public static final Double kClimberMinOutPut = -1.0;
    public static final Double kTrapMaxOutput = 1.0;

  }

  public static final class ShooterConstants {

    public static final int shooterMotor1Id = 13;
    public static final int shooterMotor2Id = 14;
    public static final int beltMotorId1 = 15;

    public static final double kshooterP = 0.0002;
    public static final double kshooterI = 0;
    public static final double kshooterD = 0.001;
    public static final double kshooterFF = 0.00019;

    public static final double shooterSpeakerSpeed = 3500;
    public static final double beltSpeakerSpeed = -1;
    public static final double revSpeakerTime = 1;
    public static final double shootSpeakerTime = .5;

    public static final double shooterAmpTopSpeed = 200;
    public static final double shooterAmpBottomSpeed = 1200;
    public static final double beltAmpSpeed = -1;
    public static final double revAmpTime = .5;
    public static final double shootAmpTime = .8;

    public static final double SourceShooterSpeed = .3;
    public static final double SourceBeltSpeed = .5;

    public static final double DefaultSpeed = 0;

  }

  public static final class TrapConstants {
    public static final double kTrapP = .03;
    public static final double kTrapI = 0;
    public static final double kTrapD = 0;
    public static final double kTrapFF = 0;
    public static final double kTrapMinOutput = -1;
    public static final double kTrapMaxOutput = 1;
    public static final double kpositionUp = 24;
    public static final double kpositionInOut = 0;
    public static final double kTrapEngaged = 150;
    public static final double kTrapDisengaged = 0;
    public static final int kUpDownMotorId = 12;
    public static final int kInOutMotorId = 20;
    public static final int kTrackMotorId = 0; // Servo

  }

  public static final class EndgameConstants {
    public static final double kTrapUpTime = 2.5;
    public static final double kTrapReleaseTime = .25;
    public static final double kclimberTime = 2;
    public static final double kTrapOutTime = .5;
    public static final double waitTime = .25;

  }

  public static final class AutoConstants {
    public static final double maxModuleSpeed = 2;
    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(2, 0, 0), new PIDConstants(2, 0, 0), maxModuleSpeed, 0.508, new ReplanningConfig(true, true)); // TODO:
                                                                                                                        // check
    // driveBaseRadius
  }

  // Fix Limelight Position Constant
  public static final class LimeLightConstants {
    public static final String cameraName = "camera";
    // distance limelight is from center of robot(m)
    public static final double xTranslation = 0.1016;
    public static final double yTranslation = 0;
    public static final double zTranslation = 0.7874;
    // Rotation of the limelight in Radians
    public static final double rollRotation = 0; // side to side
    public static final double pitchRotation = Math.toRadians(15); // up and down
    public static final double yawRotation = Math.toRadians(180);// Tilt

  }

  public static final class FieldConstants {
    public static final double middleLineX = 8.0;
    public static final Pose2d ampPoseBlue2 = new Pose2d(1.85, 7.62, Rotation2d.fromDegrees(180));
    public static final Pose2d ampPoseBlue1 = new Pose2d(1.85, 7.55, Rotation2d.fromDegrees(180));
    public static final Pose2d ampPoseRed2 = new Pose2d(14.71, 7.59, Rotation2d.fromDegrees(180));
    public static final Pose2d ampPoseRed1 = new Pose2d(14.71, 7.55, Rotation2d.fromDegrees(180));
    public static final double angleThreshold = 0; // TODO change
    public static final double speakerTargetDistance = 0; // TODO change
    public static final int blueSpeakerAprilID = 7;
    public static final int redSpeakerAprilID = 3;

  }

  public static final class AlignConstants {

    public static final double kmaxVelocityMps = 1;
    public static final double kmaxAccelerationMpsSq = 1;
    public static final double kmaxAngularVelocityRps = 1;
    public static final double kmaxAngularAccelerationRpsSq = 1;

  }

  public static final class LedConstants {
    public static final int ledLength = 176;
    public static final int ledPWMID = 1;

    public static final int shooterLedStart = 0;
    public static final int shooterLedMiddleDiv = 20;
    public static final int shooterLedEnd = 29;

    public static final int topBarLedStart = 31;
    public static final int topBarLedStop = 64;

    public static final int NoNoteRed = 252;
    public static final int NoNoteGreen = 190;
    public static final int NoNoteBlue = 3;

    public static final int YesNoteRed = 15;
    public static final int YesNoteGreen = 252;
    public static final int YesNoteBlue = 3;

  }

  public static final class IRSensorConstants {
    public static final int IRSensorID = 8;

  }
}