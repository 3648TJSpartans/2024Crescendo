package frc.robot.vision;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import frc.robot.Constants.LimeLightConstants;

public class PoseEstimator {

    PhotonPoseEstimator photonPoseEstimator;
    private AprilTagFieldLayout layout;

    public PoseEstimator() {
        Transform3d robotOnCamera = new Transform3d(
                new Translation3d(LimeLightConstants.xTranslation, LimeLightConstants.yTranslation,
                        LimeLightConstants.zTranslation),
                new Rotation3d(LimeLightConstants.rollRotation, LimeLightConstants.pitchRotation,
                        LimeLightConstants.yawRotation));
        photonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.AVERAGE_BEST_TARGETS,
                new PhotonCamera(LimeLightConstants.cameraName), robotOnCamera);
        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    public void update() {
        photonPoseEstimator.update();
    }

}
