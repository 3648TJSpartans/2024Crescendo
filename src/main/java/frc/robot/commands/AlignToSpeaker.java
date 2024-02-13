package frc.robot.commands;

import org.opencv.photo.Photo;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.vision.VisionPoseEstimator;

public class AlignToSpeaker extends Command {
    private final SwerveSubsystem m_swerveSubsystem;
    private final VisionPoseEstimator m_visionPoseEstimator;

    public AlignToSpeaker(SwerveSubsystem swerveSubsystem, VisionPoseEstimator visionPoseEstimator) {
        m_visionPoseEstimator = visionPoseEstimator;
        m_swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        PhotonPipelineResult result = m_visionPoseEstimator.getLatestResult();
        if (result.hasTargets()) {

            PhotonTrackedTarget target = result.getBestTarget();
            double distance = m_visionPoseEstimator.getDistanceToApirlTag(target, target.getFiducialId());
            if (Math.abs(target.getYaw()) > FieldConstants.angleThreshold) {
                // Turn
            }
            if (distance > FieldConstants.speakerTargetDistance) {
                // Drive
            }
        }
    }

    @Override
    public boolean isFinished() {
        PhotonPipelineResult finalResult = m_visionPoseEstimator.getLatestResult();
        if (finalResult.hasTargets()) {
            PhotonTrackedTarget target = finalResult.getBestTarget();
            double distance = m_visionPoseEstimator.getDistanceToApirlTag(target, target.getFiducialId());
            return Math.abs(target.getYaw()) < FieldConstants.angleThreshold
                    && distance <= FieldConstants.speakerTargetDistance;
        }
        return false;
    }

}
