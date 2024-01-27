// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    // Test Swerve Can:
    // public static final int kFrontLeftDrivingCanId = 4;
    // public static final int kFrontLeftTurningCanId = 3;

    // public static final int kRearLeftDrivingCanId = 2;
    // public static final int kRearLeftTurningCanId = 1;

    // public static final int kFrontRightDrivingCanId = 8;
    // public static final int kFrontRightTurningCanId = 7;

    // public static final int kRearRightDrivingCanId = 6;
    // public static final int kRearRightTurningCanId = 5;

    // Final CAN
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kFrontLeftTurningCanId = 1;

    public static final int kRearLeftDrivingCanId = 8;
    public static final int kRearLeftTurningCanId = 7;

    public static final int kFrontRightDrivingCanId = 4;
    public static final int kFrontRightTurningCanId = 3;

    public static final int kRearRightDrivingCanId = 6;
    public static final int kRearRightTurningCanId = 5;

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
    public static final int AButton = 1;
    public static final int BButton = 2;
    public static final int YButton = 3;
    public static final int XButton = 4;
    public static final int LSButton = 5;
    public static final int RSButton = 6;
    public static final int kDriverControllerPort = 0;
    public static final int kCopilotControllerPort = 1;
    public static final int kResetHeadingButton = 1;
    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 6;
    public static final int kcopilotXAxis = 0;
    public static final double kDeadband = 0.05;

  }

  public static final class IntakeConstants {
    public static final int IntakeMotor1Id = 9;
    public static final int IntakeMotor2Id = 10;
    public static final double IntakeSpeed = .65;
  }

  public static final class AutoConstants {
    public static final double maxModuleSpeed = 5;
    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(0, 0, 0), new PIDConstants(0, 0, 0), 5, 0.565685, new ReplanningConfig()); // TODO: check
                                                                                                    // driveBaseRadius
  }

  public static final class ArmConstants {
    public static final int armMotorId = 16;
    public static final int armMotorId = 20;
  }

  public static final class ClimberConstants {
    public static final int climberMotor1ID = 11;
    public static final int climberMotor2ID = 12;
    public static final double kClimberP = 0;
    public static final double kClimberI = 0;
    public static final double kClimberD = 0;
    public static final double kClimberFF = 0;
    public static final double kClimberMinOutput = 0;
    public static final double kClimberMaxOutput = 0;
  }

  public static final class ShooterConstants {
    public static final int shooterMotor1Id = 13;
    public static final int shooterMotor2Id = 11;
    public static final int beltMotorId = 12;
    public static final double motorSpeed = 0.3;
    public static final double beltMotorSpeed = 0.3;
    public static final double idleTime = 0;
  }

public static final class IRSensorConstants {

}
public static final class TrapConstants {
    public static final double kTrapP = 0;
    public static final double kTrapI = 0;
    public static final double kTrapD = 0;
    public static final double kTrapFF = 0;
    public static final double kTrapMinOutput = -1;
    public static final double kTrapMaxOutput = 1;
    public static final double kpositionUpDown = 0;
    public static final double kpositionInOut = 0;
    public static final double kspeed = 0;
    public static final int kUpDownMotorId = 0;
    public static final int kInOutMotorId = 0;
    public static final int kTrackMotorId = 0;
    public static final double kTrapTime = 2.5;
  }
}
