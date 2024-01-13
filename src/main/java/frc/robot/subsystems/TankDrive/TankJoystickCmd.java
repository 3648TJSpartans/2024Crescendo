package frc.robot.subsystems.TankDrive;

import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TankDriveConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class TankJoystickCmd extends Command {
    private final TankDrive tankSubsystem;
    private final Supplier<Double> xInputFunction, yInputFunction;
    private final Supplier<Boolean> driveModeFunction;
    private final SlewRateLimiter xLimiter, yLimiter;

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

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (driveModeFunction.get()) {
            tankSubsystem.setInputForward(MathUtil.applyDeadband(xInputFunction.get(), TankDriveConstants.kDeadzone));
            tankSubsystem.setInputTurn(MathUtil.applyDeadband(yInputFunction.get(), TankDriveConstants.kDeadzone));
        } else {
            tankSubsystem.setInputLeft(MathUtil.applyDeadband(xInputFunction.get(), TankDriveConstants.kDeadzone));
            tankSubsystem.setInputRight(MathUtil.applyDeadband(yInputFunction.get(), TankDriveConstants.kDeadzone));
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
