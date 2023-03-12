package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlignToDegreesCommand extends CommandBase {

    public static final double SPEED = Units.degreesToRadians(120);
    public static final ChassisSpeeds POS = new ChassisSpeeds(0, 0, SPEED);
    public static final ChassisSpeeds NEG = new ChassisSpeeds(0, 0, -SPEED);

    private double currentDirection;
    private final SwerveDriveSubsystem drive;
    private double directionInitial;
    private boolean done;
    private final double wantedDirection;
    private final double toleration = 1;
    private double directionDisposition;

    public AlignToDegreesCommand(SwerveDriveSubsystem drive, double wantedDirection) {

        this.drive = drive;
        this.wantedDirection = wantedDirection;

        addRequirements(drive);
    }

    @Override
    public void initialize() {

        directionInitial = drive.getYaw().getDegrees();
        directionDisposition = wantedDirection - directionInitial;
        // REPLACE ME with real logic that actually does something
        done = false;
        System.err.println(directionDisposition);
    }

    @Override
    public void execute() {

        currentDirection = drive.getYaw().getDegrees();
        System.err.println(String.format("want %.3f, have %.3f, diff %.3f",
                wantedDirection,
                currentDirection,
                wantedDirection - currentDirection));
        SmartDashboard.putNumber("getYaw", currentDirection);

        if (currentDirection < toleration + wantedDirection && currentDirection > wantedDirection - toleration){
            drive.stop();
            done = true;
        } else if (directionDisposition < -180 || (directionDisposition < 180 && directionDisposition > 0)){
            drive.drive(POS);
        } else {
            drive.drive(NEG);
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
