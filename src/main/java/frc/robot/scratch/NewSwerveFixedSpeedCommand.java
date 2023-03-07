package frc.robot.scratch;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.HalfBakedSpeedController;

/**
 * Update to our SwerveFixedSpeedCommand that corrects for rotational
 * drift using the robot's heading.
 */
public class NewSwerveFixedSpeedCommand extends CommandBase {

    public static final double MIN_THRESHOLD = 1.0;
    public static final double MAX_THRESHOLD = 5.0;
    public static final double MIN_SPEED_OMEGA = 0.1;
    public static final double MAX_SPEED_OMEGA = Units.degreesToRadians(120);

    private final SwerveDriveSubsystem drive;
    private final HalfBakedSpeedController controller;
    private final double speedX;
    private final double speedY;
    private final double duration;
    private double startTime;
    private double startHeading;
    private boolean done;

    public NewSwerveFixedSpeedCommand(
            SwerveDriveSubsystem drive,
            double speedX,
            double speedY,
            double duration) {

        this.drive = drive;
        this.speedX = speedX;
        this.speedY = speedY;
        this.duration = duration;
        this.controller = new HalfBakedSpeedController(
                MIN_THRESHOLD, MAX_THRESHOLD,
                MIN_SPEED_OMEGA, MAX_SPEED_OMEGA);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        startHeading = drive.getYaw().getDegrees();
        done = false;
    }

    @Override
    public void execute() {
        double timeElapsed = Timer.getFPGATimestamp() - startTime;
        if (timeElapsed < duration) {
            ChassisSpeeds speeds = new ChassisSpeeds(
                    speedX,
                    speedY,
                    calculateSpeedOmega());
            drive.drive(speeds);
        } else {
            drive.stop();
            done = true;
        }
    }

    private double calculateSpeedOmega() {
        double currentHeading = drive.getYaw().getDegrees();
        double errorHeading = startHeading - currentHeading;
        if (errorHeading < -180) {
            errorHeading += 180;
        }
        return controller.calculate(errorHeading);
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}