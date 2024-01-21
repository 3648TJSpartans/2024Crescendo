package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveJoystickCmd extends Command {
    private SwerveSubsystem m_swerveSubsystem;
    private Supplier<Double> m_xSpeedFunction;
    private Supplier<Double> m_ySpeedFunction;
    private Supplier<Double> m_rotFunction;
    private double m_prevTime;
    private SlewRateLimiter m_xLimiter;
    private SlewRateLimiter m_yLimiter;
    private SlewRateLimiter m_turningLimiter;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction) {
        m_swerveSubsystem = swerveSubsystem;
        m_xSpeedFunction = xSpdFunction;
        m_ySpeedFunction = ySpdFunction;
        m_rotFunction = turningSpdFunction;

        m_xLimiter = new SlewRateLimiter(DriveConstants.kMaxSpeedMetersPerSecond);
        m_yLimiter = new SlewRateLimiter(DriveConstants.kMaxSpeedMetersPerSecond);
        m_turningLimiter = new SlewRateLimiter(DriveConstants.kMaxAngularSpeed);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        m_prevTime = WPIUtilJNI.now() * 1e-6;
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = m_xSpeedFunction.get();
        double ySpeed = m_ySpeedFunction.get();
        double turningSpeed = m_rotFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = m_xLimiter.calculate(xSpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
        ySpeed = m_yLimiter.calculate(ySpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
        turningSpeed = m_turningLimiter.calculate(turningSpeed)
                * DriveConstants.kMaxAngularSpeed;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;

        if (m_swerveSubsystem.getFieldRelative()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, m_swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        m_swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveSubsystem.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
