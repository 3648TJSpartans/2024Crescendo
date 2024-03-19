// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.vision.VisionPoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public final class AlignCommands extends Command {
  private static VisionPoseEstimator m_visionPoseEstimator;
  private static SwerveSubsystem m_swerveSubsystem;

  public static Command alignToAmp(VisionPoseEstimator visionPoseEstimator) {
    m_visionPoseEstimator = visionPoseEstimator;
    Pose2d ampPose1;
    Pose2d ampPose2;
    List<Translation2d> bezierPoints;
    if (m_visionPoseEstimator.getVisionPose().getX() < FieldConstants.middleLineX) {
      ampPose2 = FieldConstants.ampPoseBlue2;
      ampPose1 = FieldConstants.ampPoseBlue1;

      bezierPoints = PathPlannerPath.bezierFromPoses(ampPose1, ampPose2);

    } else {
      ampPose2 = FieldConstants.ampPoseRed2;
      ampPose1 = FieldConstants.ampPoseRed1;
      bezierPoints = PathPlannerPath.bezierFromPoses(ampPose1, ampPose2);
    }

    PathPlannerPath path = new PathPlannerPath(
        bezierPoints,
        new PathConstraints(AlignConstants.kmaxVelocityMps, AlignConstants.kmaxAccelerationMpsSq,
            AlignConstants.kmaxAngularVelocityRps, AlignConstants.kmaxAngularAccelerationRpsSq),
        new GoalEndState(0.0, ampPose1.getRotation()));

    return AutoBuilder.followPath(path);
  }

  public static void alignToSource(VisionPoseEstimator visionPoseEstimator) {
    m_visionPoseEstimator = visionPoseEstimator;
    Pose2d sourcePose2d;
    List<Translation2d> bezierPoints;
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        sourcePose2d = FieldConstants.redSourcePose;
      } else {
        sourcePose2d = FieldConstants.blueSourcePose;

      }

    } else {
      sourcePose2d = new Pose2d();
    }
    bezierPoints = PathPlannerPath.bezierFromPoses(m_visionPoseEstimator.getVisionPose(), sourcePose2d);
    PathPlannerPath path = new PathPlannerPath(bezierPoints,
        new PathConstraints(AlignConstants.kmaxVelocityMps, AlignConstants.kmaxAccelerationMpsSq,
            AlignConstants.kmaxAngularVelocityRps, AlignConstants.kmaxAngularAccelerationRpsSq),
        new GoalEndState(0, sourcePose2d.getRotation()));
    AutoBuilder.followPath(path).schedule();
  }

  private AlignCommands() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

}
