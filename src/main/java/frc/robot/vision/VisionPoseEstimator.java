package frc.robot.vision;

import java.io.IOException;
import java.util.Arrays;
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

    private AprilTagFieldLayout layout;
    private SwerveDrivePoseEstimator m_visionPoseEstimator;
    private final SwerveSubsystem m_swerveSubsystem;
    private PhotonCamera photonCamera;
    private Transform3d m_robotOnCamera;
    private PoseStrategy m_poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    private PhotonPoseEstimator photonPoseEstimator;

    private final Pose2d[] aprilTagPoses = new Pose2d[16];

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
        photonPoseEstimator = new PhotonPoseEstimator(layout, m_poseStrategy, photonCamera, m_robotOnCamera);

        m_visionPoseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
                m_swerveSubsystem.getRotation2d(), m_swerveSubsystem.getPositions(), new Pose2d());
        for (int i = 1; i < 16; i++) {
            aprilTagPoses[i] = layout.getTagPose(i).get().toPose2d();
        }

    }

    public void update() {
        photonPoseEstimator.update().ifPresent(estimatedRobotPose -> {
            SmartDashboard.putNumber("Robot Pose X", estimatedRobotPose.estimatedPose.getX());
            SmartDashboard.putNumber("Robot Pose Y", estimatedRobotPose.estimatedPose.getY());
            m_visionPoseEstimator.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(),
                    estimatedRobotPose.timestampSeconds);
        });
    }

    public Pose2d getVisionPose() {
        return m_visionPoseEstimator.getEstimatedPosition();
    }

    public Pose2d getClosestAprilTag() {
        return getVisionPose().nearest(Arrays.asList(aprilTagPoses));
    }

}
