// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.TankDrive;

import org.w3c.dom.Element;

import frc.robot.Constants.TankDriveConstants;

//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import com.revrobotics.SparkAbsoluteEncoder.Type;

//import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.kinematics.SwerveModulePosition;
//import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.ModuleConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/* import frc.robot.framework.robot.RobotXML;
import frc.robot.framework.robot.SubsystemCollection;
import frc.robot.framework.util.ShuffleboardFramework;
import frc.robot.framework.util.ShuffleboardFramework.ShuffleboardBase;
import frc.robot.framework.util.CommandMode;
import frc.robot.framework.util.Log;
import frc.robot.framework.sensor.gyroscope.*; */

public class TankDrive extends SubsystemBase {
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

  public double input_forward = 0;
  public double input_turn = 0;
  public double input_left;
  public double input_right;

  public TankDrive() {
    leftMotor = new CANSparkMax(TankDriveConstants.kLeftDriveMotorPort, MotorType.kBrushless);
    rightMotor = new CANSparkMax(TankDriveConstants.kRightDriveMotorPort, MotorType.kBrushless);
    leftMotor.setInverted(TankDriveConstants.kLeftDriveMotorReversed);
    rightMotor.setInverted(TankDriveConstants.kRightDriveMotorReversed);
  }

  @Override
  public void periodic() {
    input_left = input_turn + input_forward;
    input_right = input_turn - input_forward;

    leftMotor.set(input_left);
    rightMotor.set(input_right);
  }

  public void setInputForward(double forward) {
    input_forward = forward;
  }

  public void setInputTurn(double turn) {
    input_turn = turn;
  }

  public void setInputLeft(double left) {
    input_left = left;
  }

  public void setInputRight(double right) {
    input_right = right;
  }

  public void stop() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
