package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class SimpleParkCommand extends CommandBase {

    public static final double THRESHOLD = 0.5;
    public static final double SPEED = 0.35;

    public static final ChassisSpeeds FORWARD = new ChassisSpeeds(SPEED, 0, 0);
    public static final ChassisSpeeds BACKWARD = new ChassisSpeeds(-SPEED, 0, 0);

    private final SwerveDriveSubsystem drive;
    private double startingPitch;
    private double currentPitch;
    private double deltaPitch;
    private boolean done;

    public SimpleParkCommand(SwerveDriveSubsystem drive) {

        this.drive = drive;

        SmartDashboard.putData("SimpleParkCommand", builder -> {
            builder.addBooleanProperty("Done", () -> done, null);
            builder.addDoubleProperty("PitchCurrent", () -> currentPitch, null);
            builder.addDoubleProperty("PitchDelta", () -> deltaPitch, null);
            builder.addDoubleProperty("PitchStart", () -> startingPitch, null);
        });

        addRequirements(drive);
    }

    public void initialize() {
        this.startingPitch = drive.getPitch();
        this.currentPitch = drive.getPitch();
        this.deltaPitch = 0;
        this.done = false;
    }

    public void execute() {
        currentPitch = drive.getPitch();
        deltaPitch = currentPitch - startingPitch;
        if (Math.abs(deltaPitch) > THRESHOLD) {
            drive.stop();
            done = true;
        } else if (startingPitch > 0) {
            drive.drive(FORWARD);
            done = false;
        } else {
            drive.drive(BACKWARD);
            done = false;
        }
    }

    public boolean isFinished() {
        return done;
    }
}
