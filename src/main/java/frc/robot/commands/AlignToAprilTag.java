package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class AlignToAprilTag extends Command {
    SwerveSubsystem m_swerveSubsystem;
    Pose2d finalDesiredPose = new Pose2d();
    PIDController xController = new PIDController(1.5, 0, 0);
    PIDController yController = new PIDController(1.5, 0, 0);

    public AlignToAprilTag(SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
        addRequirements(m_swerveSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d closetNode = m_swerveSubsystem.getClosestAprilTag();
        finalDesiredPose = closetNode.transformBy(
                new Transform2d(new Translation2d(FieldConstants.goalDistance, 0), Rotation2d.fromDegrees(0)));
    }

}
