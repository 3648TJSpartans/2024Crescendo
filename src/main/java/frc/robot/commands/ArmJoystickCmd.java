package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;



public class ArmJoystickCmd extends Command {
    private final ArmSubsystem ArmSubsystem;
    private Supplier<Double> xSpeed;
    public ArmJoystickCmd(ArmSubsystem subsystem, Supplier<Double> xSpeed ) {
        this.xSpeed = xSpeed;
        this.ArmSubsystem = subsystem;
        addRequirements(ArmSubsystem);

    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed = xSpeed.get();
        ArmSubsystem.MoveArm(speed);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
