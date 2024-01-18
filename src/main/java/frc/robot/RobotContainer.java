// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.IntakeButtonCmd;
import frc.robot.commands.ShooterCommands.ShootCommand;
import frc.robot.commands.ShooterCommands.ShooterCommandGroup;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.SolenoidCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Solenoid.SolenoidSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SendableChooser<Command> autoChooser;
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  // private final SolenoidSubsystem solenoidSubsystem = new SolenoidSubsystem();

  private final Joystick copilotJoystick = new Joystick(OIConstants.kCopilotControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    /**
     * "setDefaultCommand()" sets the command for how sets how "SolenoidCmd()" acts
     * - "SolenoidCmd()" is the constructor that was created in the SolenoidCmd.java
     * - Info about "SolenoidCmd()" is in SolenoidCmd.java documentation
     */
    // solenoidSubsystem.setDefaultCommand(new SolenoidCmd(
    // solenoidSubsystem,
    // /**
    // * Button Int Value of AButton = 1
    // * Button Int Value of XButton = 3
    // * Button Int Value of YButton = 4
    // */
    // () -> driverJoystick.getRawButton(OIConstants.AButton),
    // () -> driverJoystick.getRawButton(OIConstants.XButton),
    // () -> driverJoystick.getRawButton(OIConstants.YButton)));

    m_swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(m_swerveSubsystem,
        () -> -MathUtil.applyDeadband(driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
            OIConstants.kDeadband),
        () -> -MathUtil.applyDeadband(driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
            OIConstants.kDeadband),
        () -> -MathUtil.applyDeadband(driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
            OIConstants.kDeadband),
        true, true));
    m_intakeSubsystem.setDefaultCommand(new IntakeButtonCmd(m_intakeSubsystem, () -> driverJoystick.getRawButton(5),
        () -> driverJoystick.getRawButton(6)));
    // ArmSubsystem.setDefaultCommand(new ArmJoystickCmd(
    // ArmSubsystem,
    // () -> -ArmJoytick.getRawAxis(OIConstants.kDriverYAxis)));
    m_shooterSubsystem
        .setDefaultCommand(new ShooterCommandGroup(m_shooterSubsystem,
            () -> driverJoystick.getRawButtonPressed(OIConstants.XButton)));

    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // SmartDashboard.putData("Example Auto", Autos.followTestAuto());
    // SmartDashboard.putData("Square Auto", Autos.followSquareAuto());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return autoChooser.getSelected();

  }
}
