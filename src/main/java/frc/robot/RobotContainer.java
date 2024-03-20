// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.robot.commands.ShooterCommands.*;
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
  // The robot's subsystems and commands are defined here...
  private SendableChooser<Command> autoChooser;
  // The robot's subsystems and commands are defined here...

  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final TrapSubsystem m_trapSubsystem = new TrapSubsystem();
  private final LedsSubsystem m_ledsSubsystem = new LedsSubsystem();
  private final DigitalInput m_IRSenor = new DigitalInput(IRSensorConstants.IRSensorID);
  private final VisionPoseEstimator m_visionPoseEstimator = new VisionPoseEstimator(m_swerveSubsystem);
  private final CommandXboxController m_driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);
  private final CommandXboxController m_copilotController = new CommandXboxController(
      OIConstants.kCopilotControllerPort);

  private Trigger m_IRSenorTrigger = new Trigger(() -> m_IRSenor.get());

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
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
    m_driverController.rightBumper().onTrue(new IRIntakeCommand(m_intakeSubsystem, m_shooterSubsystem, m_IRSenor));
    m_driverController.leftBumper().onTrue(new IRSourceIntakeCmd(m_shooterSubsystem, m_IRSenor));
    // m_IRSenorTrigger.onTrue().onFalse();
  }

  private void configureShooter() {
    m_driverController.y().onTrue(new ShooterCommandGroup(m_shooterSubsystem));
    m_driverController.y().onTrue(new ShootLedCommand(m_ledsSubsystem));
    m_driverController.x().onTrue(new AmpCommandGroup(m_shooterSubsystem));

    // m_copilotController.b()
    // .onTrue(new InstantCommand(() -> m_shooterSubsystem.shuffleboardShooter()));
    // m_copilotController.x()
    // .onTrue(new InstantCommand(() ->
    // m_shooterSubsystem.setBeltSpeed(ShooterConstants.beltAmpSpeed)));
    // m_copilotController.b()
    // .onFalse(new InstantCommand(() -> m_shooterSubsystem.setShooterVelocity(0,
    // 0)));
    // m_copilotController.x()
    // .onFalse(new InstantCommand(() -> m_shooterSubsystem.setBeltSpeed(0)));
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
    NamedCommands.registerCommand("shoot", new ShooterCommandGroup(m_shooterSubsystem));
    NamedCommands.registerCommand("ampShoot", new AmpCommandGroup(m_shooterSubsystem));
    NamedCommands.registerCommand("Intake", new IRIntakeCommand(m_intakeSubsystem, m_shooterSubsystem, m_IRSenor));
    AutoBuilder.configureHolonomic(m_visionPoseEstimator::getVisionPose, m_swerveSubsystem::resetOdometry,
        m_swerveSubsystem::getSpeeds, m_swerveSubsystem::driveRobotRelative,
        AutoConstants.pathFollowerConfig, this::shouldFlipPath, m_swerveSubsystem);
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureTrap() {

    m_copilotController.b()
        .onTrue(new EndgameCmdGroup(m_trapSubsystem, m_climberSubsystem));
    m_copilotController.x()
        .toggleOnTrue(Commands.startEnd(() -> m_trapSubsystem.setUpDownPosition(TrapConstants.kpositionUp),
            () -> m_trapSubsystem.setUpDownPosition(0), m_trapSubsystem));
    m_trapSubsystem.setDefaultCommand(new TrapJoystickCmd(m_trapSubsystem,
        () -> -MathUtil.applyDeadband(m_copilotController.getRightY(),
            OIConstants.kDeadband),
        () -> -MathUtil.applyDeadband(m_copilotController.getRightX(),
            OIConstants.kDeadband)));

    // m_trapSubsystem.setDefaultCommand(new TrapJoystickCmd(m_trapSubsystem,
    // () -> -MathUtil.applyDeadband(m_copilotController.getLeftY(),
    // OIConstants.kDeadband),
    // () -> -MathUtil.applyDeadband(m_copilotController.getRightY(),
    // OIConstants.kDeadband)));
    m_copilotController.a().toggleOnTrue(
        Commands.startEnd(() -> m_trapSubsystem.setTrack(160), () -> m_trapSubsystem.setTrack(0), m_trapSubsystem));

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
    // m_ledsSubsystem.intakeColor(m_IRSenor);
    m_visionPoseEstimator.updateVisionPose();
    SmartDashboard.putNumber("PoseX", m_visionPoseEstimator.getVisionPose().getX());
    SmartDashboard.putNumber("PoseY", m_visionPoseEstimator.getVisionPose().getY());

  }
}
