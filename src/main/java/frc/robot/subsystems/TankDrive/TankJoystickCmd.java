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
    private final Supplier<Double> xInputFunction, yInputFunction;
    // private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter;

    private double deadzone = 0.1;
    private String mode;

    public TankJoystickCmd(TankDrive tankSubsystem, Supplier<Double> xInputFunction, Supplier<Double> yInputFunction,
            String mode) {
        this.tankSubsystem = tankSubsystem;
        this.xInputFunction = xInputFunction;
        this.yInputFunction = yInputFunction;
        // this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.mode = mode;
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
        if (mode == "FT") {
            tankSubsystem.setInputForward(deadZoneCheck(xInputFunction.get(), deadzone));
            tankSubsystem.setInputTurn(deadZoneCheck(yInputFunction.get(), deadzone));
        } else if (mode == "LR") {
            tankSubsystem.setInputLeft(deadZoneCheck(xInputFunction.get(), deadzone));
            tankSubsystem.setInputRight(deadZoneCheck(yInputFunction.get(), deadzone));
        } else {
            throw new NumberFormatException("String mode cannot be parsed. Enter FT or LR to continue");
        }
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
