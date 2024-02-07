// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.TrapConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TrapJoystickCmd;
import frc.robot.commands.Endgame.EndgameCmdGroup;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.commands.ClimberJoystickCmd;
import frc.robot.commands.IntakeButtonCmd;
import frc.robot.commands.ShooterCommands.RevMotorCommand;
import frc.robot.commands.ShooterCommands.ShooterCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TrapSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

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
  // The robot's subsystems and commands are defined here...

private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final CommandXboxController m_driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);
  private final CommandXboxController m_copilotController = new CommandXboxController(
      OIConstants.kCopilotControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("shoot", new ShooterCommandGroup(m_shooterSubsystem));
    m_swerveSubsystem.configAuto();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
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
    configureBindings();
    m_driverController.b().onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroHeading()));
  }

  private void configureIntake() {
    m_intakeSubsystem.setDefaultCommand(
        new IntakeButtonCmd(m_intakeSubsystem,
            () -> m_driverController.leftBumper().getAsBoolean(),
            () -> m_driverController.rightBumper().getAsBoolean()));

  }

  private void configureShooter() {
    m_copilotController.a().onTrue(new ShooterCommandGroup(m_shooterSubsystem));

    m_copilotController.b()
        .onTrue(new InstantCommand(() -> m_shooterSubsystem.shuffleboardShooter()));
    m_copilotController.x()
        .onTrue(new InstantCommand(() -> m_shooterSubsystem.shuffleboardBelts()));
    m_copilotController.b()
        .onFalse(new InstantCommand(() -> m_shooterSubsystem.revShooter(0)));
    m_copilotController.x()
        .onFalse(new InstantCommand(() -> m_shooterSubsystem.moveShooterIntake(0)));
  }

  private void configureClimber() {

    m_copilotController.leftBumper()
         .onTrue(new InstantCommand(() -> m_climberSubsystem.setClimberPosition(ClimberConstants.kClimberDown)));
    m_copilotController.rightBumper().onTrue(new InstantCommand(() -> m_climberSubsystem.setClimberPosition(5)));
  //  m_copilotController.a().onTrue(new InstantCommand(() -> m_climberSubsystem.setClimberPosition(80)));
    // m_climberSubsystem.setDefaultCommand(new ClimberJoystickCmd(m_climberSubsystem,
    // () -> -MathUtil.applyDeadband(m_copilotController.getLeftY(),
    // OIConstants.kDeadband)));
  }

  private void configureTrap() {

    // m_copilotController.b()
    //     .onTrue(new EndgameCmdGroup(m_trapSubsystem, m_climberSubsystem));
    m_copilotController.x()
        .toggleOnTrue(Commands.startEnd(() -> m_trapSubsystem.setUpDownPosition(TrapConstants.kpositionUp),
            () -> m_trapSubsystem.setUpDownPosition(0), m_trapSubsystem));

    // m_trapSubsystem.setDefaultCommand(new TrapJoystickCmd(m_trapSubsystem,
    // () -> -MathUtil.applyDeadband(m_copilotController.getLeftY(),
    // OIConstants.kDeadband),
    // () -> -MathUtil.applyDeadband(m_copilotController.getRightY(),
    // OIConstants.kDeadband)));
    m_copilotController.a().toggleOnTrue(
        Commands.startEnd(() -> m_trapSubsystem.setTrack(160), () -> m_trapSubsystem.setTrack(0), m_trapSubsystem));

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

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Amp1pieceAuto");
  }
}
