
package frc.robot.commands;

import java.util.List;
import java.util.Optional;

import org.opencv.photo.Photo;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.vision.VisionPoseEstimator;

public class AlignToStage extends Command {
    private final SwerveSubsystem m_swerveSubsystem;
    private final VisionPoseEstimator m_visionPoseEstimator;
    private int trackedID;
    private Pose3d tagPose;

    public AlignToStage(SwerveSubsystem swerveSubsystem, VisionPoseEstimator visionPoseEstimator) {
        m_visionPoseEstimator = visionPoseEstimator;
        m_swerveSubsystem = swerveSubsystem;
        addRequirements(m_swerveSubsystem);
    }

    @Override
    public void initialize() {
        trackedID = DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                ? FieldConstants.blueSpeakerAprilID
                : FieldConstants.redSpeakerAprilID;
        tagPose = m_visionPoseEstimator.getAprilTagPose3d(trackedID);

    }

    @Override
    public void execute() {
        PhotonPipelineResult result = m_visionPoseEstimator.getLatestResult();

        if (result.hasTargets()) {
            // Find the tag we want to chase
            Optional<PhotonTrackedTarget> target = result.getTargets().stream()
                    .filter(t -> t.getFiducialId() == trackedID)
                    .findFirst();
            if (target.isPresent()) {
                PhotonTrackedTarget info = target.get();
                double xTranslation = tagPose.getX()
                        - AlignConstants.targetDistance * Math.cos(Math.toRadians(info.getYaw()));
                double yTranslation = tagPose.getY()
                        - AlignConstants.targetDistance * Math.sin(Math.toRadians(info.getYaw()));
                Translation2d robotTransform = new Translation2d(xTranslation, yTranslation);
                Pose2d newPose = new Pose2d(robotTransform, Rotation2d.fromDegrees(info.getYaw()));

                List<Translation2d> bezierPoints = PathPlannerPath
                        .bezierFromPoses(m_visionPoseEstimator.getVisionPose(), newPose);
                PathPlannerPath path = new PathPlannerPath(bezierPoints,
                        new PathConstraints(AlignConstants.kmaxVelocityMps, AlignConstants.kmaxAccelerationMpsSq,
                                AlignConstants.kmaxAngularVelocityRps, AlignConstants.kmaxAngularAccelerationRpsSq),
                        new GoalEndState(0, Rotation2d.fromDegrees(info.getYaw())));
                AutoBuilder.followPath(path).schedule();
            }
        }
    }
}
