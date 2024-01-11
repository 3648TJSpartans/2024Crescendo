package frc.robot.subsystems.TankDrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.TankDrive.TankDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class TankJoystickCmd extends Command {
    private final TankDrive tankSubsystem;
    private final Supplier<Double> frontSpdFunction, turnSpdFunction;
    // private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter frontLimiter, turnLimiter;

    private final double deadzone = 0.1;

    public TankJoystickCmd(TankDrive tankSubsystem, Supplier<Double> frontSpdFunction, Supplier<Double> turnSpdFunction
    /* Supplier<Boolean> fieldOrientedFunction */) {
        this.tankSubsystem = tankSubsystem;
        this.frontSpdFunction = frontSpdFunction;
        this.turnSpdFunction = turnSpdFunction;
        // this.fieldOrientedFunction = fieldOrientedFunction;
        this.frontLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turnLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        addRequirements(tankSubsystem);
    }

    public double deadZoneCheck(double input, double deadzone) {
        if (Math.abs(input) < deadzone) {
            return input;
        } else {
            return 0.0;
        }
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        tankSubsystem.setInputForward(deadZoneCheck(frontSpdFunction.get(), deadzone));
        tankSubsystem.setInputTurn(deadZoneCheck(turnSpdFunction.get(), deadzone));
    }

    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
