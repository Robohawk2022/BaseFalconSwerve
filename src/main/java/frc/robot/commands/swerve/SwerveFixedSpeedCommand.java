package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

/**
 * Re-implementation of Kyle & Christopher's autonomous driving routine showing
 * how we can chain commands together to make interesting behaviors.
 */
public class SwerveFixedSpeedCommand extends CommandBase {
    
    private final SwerveDriveSubsystem drive;
    private final ChassisSpeeds speeds;
    private final boolean fieldRelative;
    private final double duration;
    private double startTime;
    private boolean done;

    public SwerveFixedSpeedCommand(
            SwerveDriveSubsystem drive,
            ChassisSpeeds speeds,
            boolean fieldRelative,
            double duration) {
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
    }

    @Override
    public void execute() {
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