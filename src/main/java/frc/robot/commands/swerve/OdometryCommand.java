package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.HalfBakedSpeedController;
import edu.wpi.first.math.geometry.Pose2d;

public class OdometryCommand extends CommandBase {

    private final SwerveDriveSubsystem drive;

    private double deltaX;
    private double deltaY;
    private double deltaOmega;

    private double targetX;
    private double targetY;
    private double targetOmega;

    private double errorX;
    private double errorY;
    private double errorOmega;

    private double speedX;
    private double speedY;
    
    private double speedOmega;
    
    Pose2d robotPose;
    SwerveDriveOdometry robotOdometry;

    private HalfBakedSpeedController omegaSpeedCalculator = new HalfBakedSpeedController(3, 10, 0.1, 0.3);
    private HalfBakedSpeedController swerveSpeedCalculator = new HalfBakedSpeedController(0.05, 0.5, 0.1, 1);

    

    public OdometryCommand(SwerveDriveSubsystem drive, double deltaX, double deltaY, double deltaOmega){

        this.drive = drive;

    }

    public void initialize(){

        robotPose = drive.getPose();
        targetX = robotPose.getX() + deltaX;
        targetY = robotPose.getY() + deltaY;
        targetOmega = robotPose.getRotation().getDegrees() + deltaOmega;
        
    }

    public void execute(){

        robotPose = drive.getPose();

        errorX = targetX - robotPose.getX();
        errorY = targetY - robotPose.getY();
        errorOmega = targetOmega - robotPose.getRotation().getDegrees();

        speedX = swerveSpeedCalculator.calculate(errorX);
        speedY = swerveSpeedCalculator.calculate(errorY);
        speedOmega = omegaSpeedCalculator.calculate(errorOmega);

        drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds((new ChassisSpeeds(speedX, deltaX, speedOmega)), drive.getYaw()));




    }
    
}
