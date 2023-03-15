package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.SwerveUtils;
import frc.robot.util.HalfBakedSpeedController;

/**
 * Re-implementation of Kyle & Christopher's autonomous driving routine showing
 * how we can chain commands together to make interesting behaviors.
 */
public class SwerveFixedSpeedCommand extends CommandBase {

    private final HalfBakedSpeedController omegaSpeedController;
    private final SwerveDriveSubsystem drive;
    private final ChassisSpeeds speeds;
    private final boolean fieldRelative;
    private final double duration;
    private double startTime;
    private Rotation2d dedicatedDirection;
    private Rotation2d currentDirection;
    private boolean done;

    public SwerveFixedSpeedCommand(
            SwerveDriveSubsystem drive,
            ChassisSpeeds speeds,
            boolean fieldRelative,
            double duration) {
        this.omegaSpeedController = new HalfBakedSpeedController(1, 5, 0.1, 0.4);
        this.drive = drive;
        this.speeds = speeds;
        this.duration = duration;
        this.fieldRelative = fieldRelative;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        done = false;
        dedicatedDirection = drive.getYaw();
    }

    @Override
    public void execute() {

        currentDirection = drive.getYaw();
        double error = SwerveUtils.angleError(currentDirection.getDegrees(), dedicatedDirection.getDegrees());
        speeds.omegaRadiansPerSecond = omegaSpeedController.calculate(error);

        double timeElapsed = Timer.getFPGATimestamp() - startTime;
        if (timeElapsed < duration) {
            if (fieldRelative) {
                drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getYaw()));
            } else {
                drive.drive(speeds);
            }
        } else {
            drive.stop();
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}