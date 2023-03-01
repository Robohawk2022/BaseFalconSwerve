package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class ParkingOnThePlatformCommand extends CommandBase {

    public static final double FORWARD_SPEED_FPS = Units.feetToMeters(2.0);
    public static final double PITCH_THRESHOLD = -0.8;
    public static final double PITCH_COUNT = 10;

    private final SwerveDriveSubsystem swerveDrive;
    private double thenPitch;
    private boolean done;
    private double retentionCounter;

    public ParkingOnThePlatformCommand(SwerveDriveSubsystem swerveDrive) {

        this.swerveDrive = swerveDrive;

        addRequirements(swerveDrive);
    }

    public void initialize() {
        thenPitch = Math.abs(swerveDrive.getPitch());
        done = false;
        retentionCounter = PITCH_COUNT;
    }

    public void execute() {

        double nowPitch = Math.abs(swerveDrive.getPitch());

        if (nowPitch - thenPitch < PITCH_THRESHOLD) {
            swerveDrive.stop();
            retentionCounter -= 1;
        } else {
            swerveDrive.drive(new ChassisSpeeds(FORWARD_SPEED_FPS, 0, 0));
            thenPitch = Math.abs(swerveDrive.getPitch());
        }

        if (retentionCounter == 0) {
            done = true;
        }
    }

    public boolean isFinished() {
        return done;
    }

}