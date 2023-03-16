package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class ParkCommand extends CommandBase {

    public static final double THRESHOLD = 0.5;
    public static final double SPEED = 0.35;

    public static final ChassisSpeeds FORWARD = new ChassisSpeeds(SPEED, 0, 0);
    public static final ChassisSpeeds BACKWARD = new ChassisSpeeds(-SPEED, 0, 0);

    private final SwerveDriveSubsystem drive;
    private double startingPitch;
    private double currentPitch;
    private boolean done;

    public ParkCommand(SwerveDriveSubsystem drive) {

        this.drive = drive;

        addRequirements(drive);
    }

    public void initialize() {
        this.startingPitch = drive.getPitch();
        this.done = false;
    }

    public void execute() {
        double deltaPitch = drive.getPitch() - startingPitch;
        if (Math.abs(deltaPitch) > THRESHOLD) {
            drive.stop();
            done = true;
        } else if (startingPitch > 0) {
            drive.drive(FORWARD);
        } else {
            drive.drive(BACKWARD);
        }
    }

    public boolean isFinished() {
        return done;
    }
}
