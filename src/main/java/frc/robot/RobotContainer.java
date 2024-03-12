// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.logging.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IRSensorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TrapConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TrapJoystickCmd;
import frc.robot.commands.Endgame.EndgameCmdGroup;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.commands.ClimberJoystickCmd;
import frc.robot.commands.IRIntakeCommand;
import frc.robot.commands.IRSourceIntakeCmd;
import frc.robot.commands.ShooterCommands.AmpCommandGroup;
import frc.robot.commands.ShooterCommands.ShooterCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TrapSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.vision.VisionPoseEstimator;

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

  private SendableChooser<Command> autoChooser;

  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final LedsSubsystem m_ledsSubsystem = new LedsSubsystem();
  private final DigitalInput m_IRSensor = new DigitalInput(IRSensorConstants.IRSensorID);
  private Command m_irIntakeCmd = new IRIntakeCommand(m_intakeSubsystem, m_shooterSubsystem, m_IRSensor,
      m_ledsSubsystem);
  private Command m_sourceIntakeCmd = new IRSourceIntakeCmd(m_shooterSubsystem, m_ledsSubsystem, m_IRSensor);
  private final VisionPoseEstimator m_visionPoseEstimator = new VisionPoseEstimator(m_swerveSubsystem);
  private final CommandXboxController m_driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);
  private final CommandXboxController m_copilotController = new CommandXboxController(
      OIConstants.kCopilotControllerPort);
  StructPublisher<Pose2d> loggedPose;

  public RobotContainer() {
    loggedPose = NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct).publish();
    m_ledsSubsystem.setIntakeColor(m_IRSensor);
    configAuto();
    configureSwerve();
    configureClimber();
    configureIntake();
    configureShooter();
  }

  private void configureSwerve() {
    SwerveJoystickCmd swerveJoystickCmd = new SwerveJoystickCmd(m_swerveSubsystem,
        () -> -MathUtil.applyDeadband(m_driverController.getLeftY(),
            OIConstants.kDeadband),
        () -> -MathUtil.applyDeadband(m_driverController.getLeftX(),
            OIConstants.kDeadband),
        () -> -MathUtil.applyDeadband(m_driverController.getRightX(),
            OIConstants.kDeadband));
    m_swerveSubsystem.setDefaultCommand(swerveJoystickCmd);
    m_driverController.a().onTrue(new InstantCommand(() -> m_swerveSubsystem.setFieldRelative()));
    m_driverController.b().onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroHeading()));
    // m_driverController.x().onTrue(new InstantCommand(() ->
    // AlignCommands.alignToAmp(m_visionPoseEstimator).schedule()));

  }

  private void configureIntake() {
    m_driverController.rightBumper()
        .toggleOnTrue(m_irIntakeCmd);

    m_driverController.leftBumper().toggleOnTrue(m_sourceIntakeCmd);
  }

  private void configureShooter() {
    m_driverController.y().onTrue(new ShooterCommandGroup(m_shooterSubsystem, m_ledsSubsystem, m_IRSensor));
    m_driverController.x().onTrue(new AmpCommandGroup(m_shooterSubsystem, m_ledsSubsystem, m_IRSensor));
  }

  private void configureClimber() {
    m_climberSubsystem.setDefaultCommand(
        new ClimberJoystickCmd(m_climberSubsystem, () -> -MathUtil.applyDeadband(m_copilotController.getLeftY(),
            OIConstants.kDeadband)));
    // m_copilotController.leftBumper()
    // .onTrue(new InstantCommand(() ->
    // m_climberSubsystem.setClimberPosition(ClimberConstants.kClimberDown)));
    // m_copilotController.rightBumper().onTrue(new InstantCommand(() ->
    // m_climberSubsystem.setClimberPosition(5)));
  }

  public void configAuto() {
    NamedCommands.registerCommand("shoot", new ShooterCommandGroup(m_shooterSubsystem, m_ledsSubsystem, m_IRSensor));
    NamedCommands.registerCommand("ampShoot", new AmpCommandGroup(m_shooterSubsystem, m_ledsSubsystem, m_IRSensor));
    NamedCommands.registerCommand("Intake", m_irIntakeCmd);
    AutoBuilder.configureHolonomic(m_visionPoseEstimator::getVisionPose, m_swerveSubsystem::resetOdometry,
        m_swerveSubsystem::getSpeeds, m_swerveSubsystem::driveRobotRelative,
        AutoConstants.pathFollowerConfig, this::shouldFlipPath, m_swerveSubsystem);
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Boolean shouldFlipPath() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    } else {
      return false;
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void runPeriodic() {
    loggedPose.set(m_visionPoseEstimator.getVisionPose());
    m_visionPoseEstimator.updateVisionPose();
    SmartDashboard.putBoolean("IRsensor", m_IRSensor.get());
    SmartDashboard.putNumber("PoseX", m_visionPoseEstimator.getVisionPose().getX());
    SmartDashboard.putNumber("PoseY", m_visionPoseEstimator.getVisionPose().getY());

  }
}
