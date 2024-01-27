package frc.robot.vision;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class VisionPoseEstimator {

    PhotonPoseEstimator m_photonPoseEstimator;
    private AprilTagFieldLayout layout;
    private SwerveDrivePoseEstimator m_nonVisionPoseEstimator;
    private SwerveDrivePoseEstimator m_visionPoseEstimator;
    private final SwerveSubsystem m_swerveSubsystem;
    private PhotonCamera photonCamera;
    PhotonCameraSim cameraSim;
    private Transform3d m_robotOnCamera;
    private PoseStrategy m_poseStrategy = PoseStrategy.AVERAGE_BEST_TARGETS;

    public VisionPoseEstimator(SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
        m_robotOnCamera = new Transform3d(
                new Translation3d(LimeLightConstants.xTranslation, LimeLightConstants.yTranslation,
                        LimeLightConstants.zTranslation),
                new Rotation3d(LimeLightConstants.rollRotation, LimeLightConstants.pitchRotation,
                        LimeLightConstants.yawRotation));
        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
        }

        photonCamera = new PhotonCamera(LimeLightConstants.cameraName);
        m_nonVisionPoseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
                swerveSubsystem.getRotation2d(), swerveSubsystem.getPositions(), new Pose2d());
        m_visionPoseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
                swerveSubsystem.getRotation2d(), swerveSubsystem.getPositions(), new Pose2d());

    }

    public void justUpdate(SwerveSubsystem swerve) {
        m_nonVisionPoseEstimator.update(swerve.getYaw(), swerve.getPositions());
        m_visionPoseEstimator.update(swerve.getYaw(), swerve.getPositions());
    }

    public void updateVisionPose(SwerveSubsystem swerve, Pose2d referencePose) {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            double smallestPoseDelta = 10e9;
            EstimatedRobotPose lowestDeltaPose = null;
            Translation2d robotToTarget = null;
            for (PhotonTrackedTarget target : result.getTargets()) {
                int id = target.getFiducialId();
                SmartDashboard.putNumber("id", id);
                Pose3d targetPosition = layout.getTagPose(id).get();

                // TODO check sign of pitch and maybe add pitch from gyro
                Rotation3d gyroCalculatedAngle;
                double yaw = swerve.getYaw().getRadians();
                Optional<Alliance> ally = DriverStation.getAlliance();
                if (ally.get() == Alliance.Red) {
                    gyroCalculatedAngle = new Rotation3d(0,
                            -m_robotOnCamera.getRotation().getY() + swerve.getGyro().getY(),
                            -yaw);
                } else {
                    gyroCalculatedAngle = new Rotation3d(0,
                            -m_robotOnCamera.getRotation().getY() + swerve.getGyro().getY(),
                            (yaw > 0 ? 1 : -1) * Math.PI - yaw);
                }

                Translation3d transformToTarget = target.getBestCameraToTarget().getTranslation();
                Pose3d estimatedPose = targetPosition
                        .transformBy(new Transform3d(transformToTarget, gyroCalculatedAngle).inverse())
                        .transformBy(m_robotOnCamera.inverse());

                // SmartDashboard.putNumber("estimatedPose", estimatedPose.getX());

                double poseDelta = referencePose.getTranslation()
                        .getDistance(estimatedPose.getTranslation().toTranslation2d());
                if (poseDelta < smallestPoseDelta) {
                    smallestPoseDelta = poseDelta;
                    lowestDeltaPose = new EstimatedRobotPose(estimatedPose, result.getTimestampSeconds(),
                            List.of(target), m_poseStrategy);
                    robotToTarget = transformToTarget.toTranslation2d();
                }
            }

            if (robotToTarget == null)
                return;
            double tagDistance = robotToTarget.getNorm();

            m_visionPoseEstimator.addVisionMeasurement(lowestDeltaPose.estimatedPose.toPose2d(),
                    lowestDeltaPose.timestampSeconds);
            SmartDashboard.putNumber("Pos3d X:", lowestDeltaPose.estimatedPose.getX());
            SmartDashboard.putNumber("Pos3d Y:", lowestDeltaPose.estimatedPose.getY());
            SmartDashboard.putNumber("Pos3d Z:", lowestDeltaPose.estimatedPose.getZ());

        }
        SmartDashboard.putNumber("time", result.getTimestampSeconds());
    }

    public Pose2d getVisionPose() {
        return m_visionPoseEstimator.getEstimatedPosition();
    }

}
