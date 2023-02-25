package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.AprilTag;

public class AlignToAprilTagCommand extends CommandBase {

    public static final double TARGET_Z = 3;
    public static final double TOLERANCE = 0.05;
    public static final double LOW_SPEED_BREAK = 0.1;
    public static final double HIGH_SPEED = 0.1;
    public static final double LOW_SPEED = 0.8;

    private final SwerveDriveSubsystem swerveDrive;
    private final VisionSubsystem vision;
    private boolean done;

    public AlignToAprilTagCommand(SwerveDriveSubsystem swerveDrive, VisionSubsystem vision) {

        this.swerveDrive = swerveDrive;
        this.vision = vision;

        addRequirements(swerveDrive);
        // we don't need to require vision, because we're not commanding it to do anything
    }

    public void initialize() {
        done = false;
    }

    private double determineSpeedX(AprilTag tag) {
        double distance = tag.getForwardReverseDistance();
        return determineSpeed(distance);
    }

    private double determineSpeedY(AprilTag tag) {
        double distance = tag.getLeftRightDistance();
        return determineSpeed(distance);
    }

    private double determineSpeed(double distance) {
        double absoluteError = Math.abs(distance - TARGET_Z);
        if (absoluteError < TOLERANCE) {
            return 0.0;
        }
        double val = absoluteError < LOW_SPEED_BREAK ? LOW_SPEED : HIGH_SPEED;
        if (distance < TARGET_Z) {
            val *= -1.0;
        }
        return val;
    }

    public void execute() {

        AprilTag tag = vision.getAprilTag();

        // If there is no tag in view, we'll just stop and be done.
        // TODO is it possible this is a transient error, and we should keep trying?
        if (tag == null) {
            swerveDrive.stop();
            done = true;
            return;
        }

        double vx = determineSpeedX(tag);
        double vy = determineSpeedY(tag);

        if (vx == 0 && vy == 0) {
            System.err.println("close enough; stopping");
            swerveDrive.stop();
            done = true;
            return;
        }

        ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, 0.0);    
        System.err.println("got tag "+tag+"; applying speed = "+speeds);
        swerveDrive.drive(speeds);
        done = false;
    }

    public boolean isFinished() {
        return done;
    }
}
