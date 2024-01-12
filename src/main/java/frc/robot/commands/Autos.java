// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public final class Autos extends Command {
  /** Example static factory for an autonomous command. */

  public static Command followTestAuto() {
    System.out.println("working");
    return new PathPlannerAuto("TestAuto");
  }

  public static Command followSquareAuto() {
    return new PathPlannerAuto("Square Auto");
  }

  // You must wrap the path following command in a FollowPathWithEvents command in
  // order for event markers to work

  // You must wrap the path following command in a FollowPathWithEvents command in
  // order for event markers to work

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

}
