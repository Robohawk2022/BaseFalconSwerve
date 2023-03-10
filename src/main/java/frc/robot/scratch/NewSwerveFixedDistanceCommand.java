package frc.robot.scratch;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.HalfBakedSpeedController;

public class NewSwerveFixedDistanceCommand extends CommandBase {

    public static final double MIN_THRESHOLD_XY = 0.1;
    public static final double MAX_THRESHOLD_XY = 1.0;
    public static final double MIN_SPEED_XY = 0.1;
    public static final double MAX_SPEED_XY = 4.0;

    public static final double MIN_THRESHOLD_OMEGA = 1.0;
    public static final double MAX_THRESHOLD_OMEGA = 5.0;
    public static final double MIN_SPEED_OMEGA = 0.1;
    public static final double MAX_SPEED_OMEGA = Units.degreesToRadians(120);

    private final SwerveDriveSubsystem drive;
    private final double dx;
    private final double dy;
    private final HalfBakedSpeedController controllerXY;
    private final HalfBakedSpeedController controllerOmega;
    private double targetX;
    private double targetY;
    private double targetHeading;
    private boolean done;

    public NewSwerveFixedDistanceCommand(SwerveDriveSubsystem drive, double dx, double dy) {

        this.drive = drive;
        this.dx = dx;
        this.dy = dy;
        this.controllerXY = new HalfBakedSpeedController(MIN_THRESHOLD_XY, MAX_THRESHOLD_XY, MIN_SPEED_XY, MAX_SPEED_XY);
        this.controllerOmega = new HalfBakedSpeedController(MIN_THRESHOLD_OMEGA, MAX_THRESHOLD_OMEGA, MIN_SPEED_OMEGA, MAX_SPEED_OMEGA);

        addRequirements(drive);
    }

    @Override
    public void initialize() {

        Pose2d pose = drive.getPose();
        targetX = pose.getX() + dx;
        targetY = pose.getY() + dy;
        targetHeading = pose.getRotation().getDegrees();

        done = false;
    }

    @Override
    public void execute() {

        Pose2d pose = drive.getPose();

        double errorx = targetX - pose.getX();
        double errory = targetY - pose.getY();
        double erroro = targetHeading - pose.getRotation().getDegrees();

        double vx = controllerXY.calculate(errorx);
        double vy = controllerXY.calculate(errory);
        double vo = controllerOmega.calculate(erroro);

        if (vx == 0 && vy == 0 && vo == 0) {
            drive.stop();
            done = true;
        } else {
            drive.drive(new ChassisSpeeds(vx, vy, vo));
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
