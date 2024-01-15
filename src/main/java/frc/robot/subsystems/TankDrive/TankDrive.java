// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.TankDrive;

import frc.robot.Constants.TankDriveConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDrive extends SubsystemBase {
    // Initializes two motors on each side
    private CANSparkMax leftMotor1, leftMotor2, rightMotor1, rightMotor2;

    // Initializes forward, turning, left side and right side input vars
    public double input_forward = 0;
    public double input_turn = 0;
    public double input_left = 0;
    public double input_right = 0;

    public TankDrive() {
        // Defines motors from ports/ids from consts
        leftMotor1 = new CANSparkMax(TankDriveConstants.kLeftDriveMotorPort1, MotorType.kBrushless);
        leftMotor2 = new CANSparkMax(TankDriveConstants.kLeftDriveMotorPort2, MotorType.kBrushless);
        rightMotor1 = new CANSparkMax(TankDriveConstants.kRightDriveMotorPort1, MotorType.kBrushless);
        rightMotor2 = new CANSparkMax(TankDriveConstants.kRightDriveMotorPort2, MotorType.kBrushless);
        leftMotor1.setInverted(TankDriveConstants.kLeftDriveMotorReversed1);
        leftMotor2.setInverted(TankDriveConstants.kLeftDriveMotorReversed2);
        rightMotor1.setInverted(TankDriveConstants.kRightDriveMotorReversed1);
        rightMotor2.setInverted(TankDriveConstants.kRightDriveMotorReversed2);
    }

    @Override
    public void periodic() {
        // Defines fwd as left/right side moving in same direction, and turn as
        // left/right side moving in opp direction
        input_left = input_turn + input_forward;
        input_right = input_turn - input_forward;

        // Sets motor speeds to left/right input
        leftMotor1.set(input_left);
        leftMotor2.set(input_left);
        rightMotor1.set(input_right);
        rightMotor2.set(input_right);
    }

    // Methods for setting fwd/turn/left/right input
    public void setInputForward(double forward) {
        input_forward = forward;
    }

    public void setInputTurn(double turn) {
        input_turn = turn;
    }

    public void setInputLeft(double left) {
        leftMotor1.set(left);
        leftMotor2.set(left);
    }

    public void setInputRight(double right) {
        rightMotor1.set(-right);
        rightMotor2.set(-right);
    }

    // Method for stopping all motors
    public void stop() {
        leftMotor1.set(0);
        leftMotor2.set(0);
        rightMotor1.set(0);
        rightMotor2.set(0);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
