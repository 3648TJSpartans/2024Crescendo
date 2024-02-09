// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public final class AlignCommands extends Command {
  /** Example static factory for an autonomous command. */

  public static Command alignToAmp() {
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(

        new Pose2d(1.85, 7.25, Rotation2d.fromDegrees(90)));

    PathPlannerPath path = new PathPlannerPath(
        bezierPoints,
        new PathConstraints(AlignConstants.kmaxVelocityMps, AlignConstants.kmaxAccelerationMpsSq,
            AlignConstants.kmaxAngularVelocityRps, AlignConstants.kmaxAngularAccelerationRpsSq),
        new GoalEndState(0.0, Rotation2d.fromDegrees(-90)));

    return AutoBuilder.followPath(path);
  }

  public static Command alignToSpeakerMiddle() {
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(

        new Pose2d(1.47, 5.54, Rotation2d.fromDegrees(180)));
    PathPlannerPath path = new PathPlannerPath(
        bezierPoints,
        new PathConstraints(AlignConstants.kmaxVelocityMps, AlignConstants.kmaxAccelerationMpsSq,
            AlignConstants.kmaxAngularVelocityRps, AlignConstants.kmaxAngularAccelerationRpsSq),
        new GoalEndState(0.0, Rotation2d.fromDegrees(180)));

    return AutoBuilder.followPath(path);

  }

  private AlignCommands() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

}
