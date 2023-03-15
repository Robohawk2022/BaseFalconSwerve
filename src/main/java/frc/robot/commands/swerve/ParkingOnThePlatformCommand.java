package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.HalfBakedSpeedController;

public class ParkingOnThePlatformCommand extends CommandBase {

    private final SwerveDriveSubsystem swerveDrive;
    
    private final double TIME_COUNT = 1.0;
    private final double SPEED = 0.5;
    private final double DELTA_THRESHOLD = 1;


    private boolean done;
    private double timeBalanced;
    private double retentionCounter;

    private double thenPitch;
    private double nowPitch;
    private double deltaPitch;

    private HalfBakedSpeedController speedReduction = new HalfBakedSpeedController(0.1, DELTA_THRESHOLD, 0.1, SPEED);

    public ParkingOnThePlatformCommand(SwerveDriveSubsystem swerveDrive) {

        this.swerveDrive = swerveDrive;

        addRequirements(swerveDrive);
    }

    public void initialize() {

        thenPitch = Math.abs(swerveDrive.getPitch());
        done = false;


    }

    public void execute() {

        nowPitch = Math.abs(swerveDrive.getPitch());
        deltaPitch = nowPitch - thenPitch;

        double speed = SPEED - speedReduction.calculate(Math.abs(deltaPitch));
       
        if (Math.abs(deltaPitch) > DELTA_THRESHOLD){

            swerveDrive.stop();
            done = true;

        }
        else if (swerveDrive.getPitch() > 0){

            swerveDrive.drive(speed, 0, 0);

        } else {

            swerveDrive.drive(-speed, 0, 0);

        }
                    
    
        

        thenPitch = nowPitch;
    }

    public boolean isFinished() {

        return done;

    }

}