package frc.robot.subsystems.Swerve;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.vision.VisionPoseEstimator;

public class SwerveSubsystem extends SubsystemBase {
    // define all modules
    private final SwerveModule m_frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset);

    private final SwerveModule m_frontRight = new SwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset);

    private final SwerveModule m_rearLeft = new SwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset);

    private final SwerveModule m_rearRight = new SwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset);
    private SwerveModule[] modules;
    private VisionPoseEstimator visionPoseEstimation;
    private SwerveDrivePoseEstimator m_swervePoseEstimator;
    private boolean isFieldRelative = false;
    // The gyro sensor
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
    // Slew rate filter variables for controlling acceleration
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(m_gyro.getAngle()),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_rearLeft.getPosition(),
                    m_rearRight.getPosition()
            });

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        visionPoseEstimation = new VisionPoseEstimator();
        m_swervePoseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
                this.getRotation2d(), this.getPositions(), new Pose2d());
        modules = new SwerveModule[] { m_frontLeft, m_frontRight, m_rearLeft, m_rearRight };

    }

    public void configAuto() {
        AutoBuilder.configureHolonomic(this::getVisionPose, this::resetOdometry,
                this::getSpeeds, this::driveRobotRelative,
                AutoConstants.pathFollowerConfig, this::shouldFlipPath, this);
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
                Rotation2d.fromDegrees(m_gyro.getAngle()),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                });
        updatePoseEstimation();
        SmartDashboard.putNumber("Gyro Pose X:", getPose().getX());
        SmartDashboard.putNumber("Gyro Pose Y:", getPose().getY());
        SmartDashboard.putNumber("New Estimated Pose X", getVisionPose().getX());
        SmartDashboard.putNumber("New Estimated Pose Y", getVisionPose().getY());
        SmartDashboard.putNumber("New Estimated Pose X Graph", getVisionPose().getX());
        SmartDashboard.putNumber("New Estimated Pose Y Graph", getVisionPose().getY());
        Optional<Alliance> ally = DriverStation.getAlliance();
        SmartDashboard.putString("Alliance Color", ally.toString());

    }

    public void updatePoseEstimation() {
        visionPoseEstimation.getPhotonPoseEstimator().update().ifPresent(estimatedRobotPose -> {
            m_swervePoseEstimator.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(),
                    estimatedRobotPose.timestampSeconds);
        });
        m_swervePoseEstimator.update(this.getYaw(),
                this.getPositions());
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public Pose2d getVisionPose() {
        return m_swervePoseEstimator.getEstimatedPosition();
    }

    public Boolean shouldFlipPath() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        } else {
            return false;
        }
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
                Rotation2d.fromDegrees(m_gyro.getAngle()),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                pose);
    }

    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
        SmartDashboard.putNumber("Front Left Encoder Value", m_frontLeft.getAbsoluteEncoder());
        SmartDashboard.putNumber("Front Right Encoder Value", m_frontRight.getAbsoluteEncoder());
        SmartDashboard.putNumber("Back Left Encoder Value", m_rearLeft.getAbsoluteEncoder());
        SmartDashboard.putNumber("Back Right Encoder Value", m_rearRight.getAbsoluteEncoder());

    }

    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }

    public void setFieldRelative() {
        isFieldRelative = !isFieldRelative;
    }

    public boolean getFieldRelative() {
        return isFieldRelative;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
        };
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    // public double getHeading() {
    // return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
    // }

    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle() + 360, 360);
    }

    public Rotation2d getRotation2d() {
        SmartDashboard.putNumber("Heading", getHeading());
        return Rotation2d.fromDegrees(-getHeading());
    }

    public ChassisSpeeds getSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    public void stopModules() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_rearLeft.stop();
        m_rearRight.stop();
    }

    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(-m_gyro.getYaw());
    }

}
