package frc.robot.vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class VisionPoseEstimator {

    PhotonPoseEstimator photonPoseEstimator;
    private AprilTagFieldLayout layout;
    private SwerveDrivePoseEstimator m_nonVisionPoseEstimator;
    private SwerveDrivePoseEstimator m_visionPoseEstimator;
    private final SwerveSubsystem m_swerveSubsystem;

    public VisionPoseEstimator(SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
        Transform3d robotOnCamera = new Transform3d(
                new Translation3d(LimeLightConstants.xTranslation, LimeLightConstants.yTranslation,
                        LimeLightConstants.zTranslation),
                new Rotation3d(LimeLightConstants.rollRotation, LimeLightConstants.pitchRotation,
                        LimeLightConstants.yawRotation));
        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
        }
        m_nonVisionPoseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
                swerveSubsystem.getRotation2d(), swerveSubsystem.getPositions(), new Pose2d());
        m_visionPoseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
                swerveSubsystem.getRotation2d(), swerveSubsystem.getPositions(), new Pose2d());
        photonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.AVERAGE_BEST_TARGETS,
                new PhotonCamera(LimeLightConstants.cameraName), robotOnCamera);

    }

    public void justUpdate(SwerveSubsystem swerve) {
        m_nonVisionPoseEstimator.update(swerve.getYaw(), swerve.getPositions());
        m_visionPoseEstimator.update(swerve.getYaw(), swerve.getPositions());
    }

    public void getVisionPose() {
        // m_visionPoseEstimator.addVisionMeasurement(lowestDeltaPose.estimatedPose.toPose2d(),
        // lowestDeltaPose.timestampSeconds, VecBuilder.fill(xyStdDev, xyStdDev, 0));

    }

}
