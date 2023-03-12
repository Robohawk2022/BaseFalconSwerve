package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.HalfBakedSpeedController;
import edu.wpi.first.math.geometry.Pose2d;

public class OdometryCommand extends CommandBase {

    private final SwerveDriveSubsystem drive;

    private final HalfBakedSpeedController omegaSpeedCalculator;
    private final HalfBakedSpeedController swerveSpeedCalculator;
    private final double deltaX;
    private final double deltaY;
    private final double deltaOmega;
    private double targetX;
    private double targetY;
    private double targetOmega;
    private double errorX;
    private double errorY;
    private double errorOmega;
    private double speedX;
    private double speedY;
    private double speedOmega;
    private Pose2d robotPose;
    private boolean done;


    public OdometryCommand(SwerveDriveSubsystem drive, double deltaX, double deltaY, double deltaOmega) {

        this.omegaSpeedCalculator = new HalfBakedSpeedController(3, 10, 0.1, 0.3);
        this.swerveSpeedCalculator = new HalfBakedSpeedController(0.05, 0.5, 0.1, 1);
        this.drive = drive;
        this.deltaX = deltaX;
        this.deltaY = deltaY;
        this.deltaOmega = deltaOmega;

        addRequirements(drive);

    }

    public void initialize(){
        robotPose = drive.getPose();
        targetX = robotPose.getX() + deltaX;
        targetY = robotPose.getY() + deltaY;
        targetOmega = robotPose.getRotation().getDegrees() + deltaOmega;
        done = false;
    }

    public void execute() {

        robotPose = drive.getPose();

        errorX = robotPose.getX() - targetX;
        errorY = robotPose.getY() - targetY;
        errorOmega = robotPose.getRotation().getDegrees() - targetOmega;

        speedX = swerveSpeedCalculator.calculate(errorX);
        speedY = swerveSpeedCalculator.calculate(errorY);
        speedOmega = omegaSpeedCalculator.calculate(errorOmega);

        if (speedX == 0 && speedY == 0 && speedOmega == 0) {
            drive.stop();
            done = true;
        } else {
            ChassisSpeeds speeds = new ChassisSpeeds(speedX, speedY, speedOmega);
            drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getYaw()));
        }
    }

    public boolean isFinished(){

        return done;

    }
}
