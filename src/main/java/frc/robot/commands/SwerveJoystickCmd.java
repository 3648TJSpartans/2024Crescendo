package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Utils.SwerveUtils;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveJoystickCmd extends Command {
    private SwerveSubsystem swerveSubsystem;
    private Supplier<Double> xSpeedFunction;
    private Supplier<Double> ySpeedFunction;
    private Supplier<Double> rotFunction;
    private Supplier<Boolean> resetFieldRelative;
    // private boolean fieldRelative;
    private boolean rateLimit;
    private double m_currentTranslationMag = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentRotation = 0.0;
    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime;
    double xSpeedCommanded;
    double ySpeedCommanded;
    private SlewRateLimiter xLimiter;
    private SlewRateLimiter yLimiter;
    private SlewRateLimiter turningLimiter;

    // public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, Supplier<Double>
    // xSpeed, Supplier<Double> ySpeed,
    // Supplier<Double> rot,
    // boolean fieldRelative, boolean rateLimit) {
    // this.swerveSubsystem = swerveSubsystem;
    // this.xSpeed = xSpeed;
    // this.ySpeed = ySpeed;
    // this.rot = rot;
    // this.fieldRelative = fieldRelative;
    // this.rateLimit = rateLimit;
    // addRequirements(swerveSubsystem);
    // }

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeedFunction = xSpdFunction;
        this.ySpeedFunction = ySpdFunction;
        this.rotFunction = turningSpdFunction;

        this.xLimiter = new SlewRateLimiter(DriveConstants.kMaxSpeedMetersPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kMaxSpeedMetersPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kMaxAngularSpeed);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        m_prevTime = WPIUtilJNI.now() * 1e-6;
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = xSpeedFunction.get();
        double ySpeed = ySpeedFunction.get();
        double turningSpeed = rotFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kMaxAngularSpeed;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;

        if (swerveSubsystem.getFeildRelative()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
        // if (resetFieldRelative.get()) {

        // }
    }

    // @Override
    // public void execute() {
    // SmartDashboard.putNumber("xSpeed", xSpeed.get());
    // SmartDashboard.putNumber("ySpeed", ySpeed.get());
    // SmartDashboard.putNumber("rot", rot.get());
    // if (rateLimit) {
    // // Convert XY to polar for rate limiting
    // double inputTranslationDir = Math.atan2(ySpeed.get(), xSpeed.get());
    // double inputTranslationMag = Math.sqrt(Math.pow(xSpeed.get(), 2) +
    // Math.pow(ySpeed.get(), 2));

    // // Calculate the direction slew rate based on an estimate of the lateral
    // // acceleration
    // double directionSlewRate;
    // if (m_currentTranslationMag != 0.0) {
    // directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate /
    // m_currentTranslationMag);
    // } else {
    // directionSlewRate = 500.0; // some high number that means the slew rate is
    // effectively instantaneous
    // }

    // double currentTime = WPIUtilJNI.now() * 1e-6;
    // double elapsedTime = currentTime - m_prevTime;
    // double angleDif = SwerveUtils.AngleDifference(inputTranslationDir,
    // m_currentTranslationDir);
    // if (angleDif < 0.45 * Math.PI) {
    // m_currentTranslationDir =
    // SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
    // directionSlewRate * elapsedTime);
    // m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
    // } else if (angleDif > 0.85 * Math.PI) {
    // if (m_currentTranslationMag > 1e-4) { // some small number to avoid
    // floating-point errors with equality
    // // checking
    // // keep currentTranslationDir unchanged
    // m_currentTranslationMag = m_magLimiter.calculate(0.0);
    // } else {
    // m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir +
    // Math.PI);
    // m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
    // }
    // } else {
    // m_currentTranslationDir =
    // SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
    // directionSlewRate * elapsedTime);
    // m_currentTranslationMag = m_magLimiter.calculate(0.0);
    // }
    // m_prevTime = currentTime;

    // xSpeedCommanded = m_currentTranslationMag *
    // Math.cos(m_currentTranslationDir);
    // ySpeedCommanded = m_currentTranslationMag *
    // Math.sin(m_currentTranslationDir);
    // m_currentRotation = m_rotLimiter.calculate(rot.get());

    // } else {
    // xSpeedCommanded = xSpeed.get();
    // ySpeedCommanded = ySpeed.get();
    // m_currentRotation = rot.get();
    // }

    // // Convert the commanded speeds into the correct units for the drivetrain
    // double xSpeedDelivered = xSpeedCommanded *
    // DriveConstants.kMaxSpeedMetersPerSecond;
    // double ySpeedDelivered = ySpeedCommanded *
    // DriveConstants.kMaxSpeedMetersPerSecond;
    // double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    // SwerveModuleState[] swerveModuleStates =
    // DriveConstants.kDriveKinematics.toSwerveModuleStates(
    // fieldRelative
    // ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered,
    // rotDelivered,
    // Rotation2d.fromDegrees(swerveSubsystem.getGyroAngle(IMUAxis.kZ)))
    // : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    // SwerveDriveKinematics.desaturateWheelSpeeds(
    // swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    // // frontLeft.setDesiredState(swerveModuleStates[0]);
    // // frontRight.setDesiredState(swerveModuleStates[1]);
    // // rearLeft.setDesiredState(swerveModuleStates[2]);
    // // rearRight.setDesiredState(swerveModuleStates[3]);
    // swerveSubsystem.setModuleStates(swerveModuleStates);

    // }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
