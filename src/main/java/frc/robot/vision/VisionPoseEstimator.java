package frc.robot.vision;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class VisionPoseEstimator {

    private AprilTagFieldLayout layout;

    private PhotonCamera photonCamera;
    private Transform3d m_robotOnCamera;
    private PoseStrategy m_poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    private PhotonPoseEstimator m_photonPoseEstimator;
    private final SwerveSubsystem m_swerveSubsystem;
    private final SwerveDrivePoseEstimator m_swervePoseEstimator;

    public VisionPoseEstimator(SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
        m_swervePoseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
                m_swerveSubsystem.getRotation2d(), m_swerveSubsystem.getPositions(), new Pose2d());
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
        m_photonPoseEstimator = new PhotonPoseEstimator(layout, m_poseStrategy, photonCamera, m_robotOnCamera);

    }

    public void updateVisionPose() {
        m_photonPoseEstimator.update().ifPresent(estimatedRobotPose -> {
            m_swervePoseEstimator.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(),
                    estimatedRobotPose.timestampSeconds);
        });
        m_swervePoseEstimator.update(m_swerveSubsystem.getYaw(),
                m_swerveSubsystem.getPositions());
    }

    public Pose2d getVisionPose() {
        return m_swervePoseEstimator.getEstimatedPosition();
    }

    public PhotonPipelineResult getLatestResult() {
        return photonCamera.getLatestResult();

    }

    public double getDistanceToApirlTag(PhotonTrackedTarget target, int ID) {
        Optional<Pose3d> tagPose = layout.getTagPose(ID);
        double distance = PhotonUtils.calculateDistanceToTargetMeters(
                LimeLightConstants.zTranslation,
                tagPose.get().getZ(),
                LimeLightConstants.pitchRotation,
                Units.degreesToRadians(target.getPitch()));
        return distance;

    }

}
