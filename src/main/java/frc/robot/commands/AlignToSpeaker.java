package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.vision.VisionPoseEstimator;

public class AlignToSpeaker extends Command {
    private final SwerveSubsystem m_swerveSubsystem;
    private final VisionPoseEstimator m_visionPoseEstimator;
    private int trackedID;
    private PhotonTrackedTarget lastTarget;

    public AlignToSpeaker(SwerveSubsystem swerveSubsystem, VisionPoseEstimator visionPoseEstimator) {
        m_visionPoseEstimator = visionPoseEstimator;
        m_swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        lastTarget = null;
        trackedID = DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                ? FieldConstants.blueSpeakerAprilID
                : FieldConstants.redSpeakerAprilID;
    }

    @Override
    public void execute() {

        PhotonPipelineResult result = m_visionPoseEstimator.getLatestResult();
        if (result.hasTargets()) {
            // PhotonTrackedTarget target = result.getBestTarget();
            var target = result.getTargets().stream()
                    .filter(t -> t.getFiducialId() == trackedID)
                    .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
                    .findFirst();
            if (target.isPresent()) {
                lastTarget = target.get();
                double distance = m_visionPoseEstimator.getDistanceToAprilTag(target.get(),
                        target.get().getFiducialId());
                if (Math.abs(target.get().getYaw()) > FieldConstants.angleThreshold) {
                    // Turn
                }
                if (distance > FieldConstants.speakerTargetDistance) {
                    // Drive
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        PhotonPipelineResult finalResult = m_visionPoseEstimator.getLatestResult();
        if (finalResult.hasTargets()) {
            PhotonTrackedTarget target = finalResult.getBestTarget();
            double distance = m_visionPoseEstimator.getDistanceToAprilTag(target, target.getFiducialId());
            return Math.abs(target.getYaw()) < FieldConstants.angleThreshold
                    && distance <= FieldConstants.speakerTargetDistance;
        }
        return false;
    }

}
