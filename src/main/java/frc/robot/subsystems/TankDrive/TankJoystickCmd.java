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
    private final Supplier<Boolean> driveModeFunction;
    private final SlewRateLimiter xLimiter, yLimiter;

    private double deadzone = 0.1;

    public TankJoystickCmd(TankDrive tankSubsystem, Supplier<Double> xInputFunction, Supplier<Double> yInputFunction,
            Supplier<Boolean> driveModeFunction) {
        this.tankSubsystem = tankSubsystem;
        this.xInputFunction = xInputFunction;
        this.yInputFunction = yInputFunction;
        this.driveModeFunction = driveModeFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
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
        if (driveModeFunction.get()) {
            tankSubsystem.setInputForward(deadZoneCheck(xInputFunction.get(), deadzone));
            tankSubsystem.setInputTurn(deadZoneCheck(yInputFunction.get(), deadzone));
        } else {
            tankSubsystem.setInputLeft(deadZoneCheck(xInputFunction.get(), deadzone));
            tankSubsystem.setInputRight(deadZoneCheck(yInputFunction.get(), deadzone));
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
