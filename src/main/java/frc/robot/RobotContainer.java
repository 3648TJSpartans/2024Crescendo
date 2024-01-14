// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SolenoidCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Solenoid.SolenoidSubsystem;
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
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
  
  private final SolenoidSubsystem solenoidSubsystem = new SolenoidSubsystem();

  XboxController exampleController = new XboxController(1);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
        () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
        () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
        () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

    /** "setDeaultCommand()" sets the command for how sets how "SolenoidCmd()" acts
        - "SolenoidCmd()" is the constructor that was created in the SolenoidCmd.java
          - Info about "SolenoidCmd()" is in SolenoidCmd.java documentation
     */
    solenoidSubsystem.setDefaultCommand(new SolenoidCmd(
      solenoidSubsystem,
      /**
      Button Int Value of 1 = "A" button 
      Button Int Value of 3 = "X" button
      Button Int Value of 4 = "Y" button
       */ 
      () -> driverJoytick.getRawButton(1),
      () -> driverJoytick.getRawButton(3),
      () -> driverJoytick.getRawButton(4)));
    configureBindings();
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
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    // new JoystickButton(driverJoytick, 2).butt(() ->
    // swerveSubsystem.zeroHeading());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // // An example command will be run in autonomous
  // return Autos.exampleAuto(m_exampleSubsystem);
  // }
}
