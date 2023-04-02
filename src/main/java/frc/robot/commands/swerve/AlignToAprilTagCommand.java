package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.AprilTag;
import frc.robot.util.HalfBakedSpeedController;

public class AlignToAprilTagCommand extends CommandBase {

    public static final double TARGET_Z = 3;
    public static final int MAX_ROUND_MISSES = 3;
    public static final double MIN_THRESHOLD = Units.feetToMeters(1);
    public static final double MAX_THRESHOLD = Units.feetToMeters(2);
    public static final double MIN_SPEED = Units.feetToMeters(0.75);
    public static final double MAX_SPEED = Units.feetToMeters(2);

    private final SwerveDriveSubsystem swerveDrive;
    private final VisionSubsystem vision;
    private final HalfBakedSpeedController controller;
    private boolean done;
    private double distanceX;
    private double errorX;
    private double outputX;
    private double distanceY;
    private double outputY;
    private int roundsWithoutTag;

    public AlignToAprilTagCommand(SwerveDriveSubsystem swerveDrive, VisionSubsystem vision) {

        this.swerveDrive = swerveDrive;
        this.vision = vision;
        this.controller = new HalfBakedSpeedController(
                MIN_THRESHOLD, MAX_THRESHOLD,
                MIN_SPEED, MAX_SPEED);
        this.roundsWithoutTag = 0;

        addRequirements(swerveDrive);

        SmartDashboard.putData("AlignToAprilTagCommand", builder -> {
            builder.addBooleanProperty("Done?", () -> done, null);
            builder.addDoubleProperty("X-Distance", () -> distanceX, null);
            builder.addDoubleProperty("X-Error", () -> errorX, null);
            builder.addDoubleProperty("X-Output", () -> outputX, null);
            builder.addDoubleProperty("Y-Distance", () -> distanceY, null);
            builder.addDoubleProperty("Y-Output", () -> outputY, null);
        });
    }

    public void initialize() {
        done = false;
        distanceX = 0;
        errorX = 0;
        outputX = 0;
        distanceY = 0;
        outputY = 0;
        roundsWithoutTag = 0;
    }

    public void execute() {

        // if there's no tag in view, we'll increment a counter; once we haven't seen it
        // for the specified number of frames, we'll assume we're done
        AprilTag tag = vision.getAprilTag();
        if (tag == null || tag.id == 0) {
            roundsWithoutTag += 1;
            System.err.println("we've been "+roundsWithoutTag+" rounds without!");
            if (roundsWithoutTag == MAX_ROUND_MISSES) {
                swerveDrive.stop();
                done = true;
            }
            return;
        }

        roundsWithoutTag = 0;

       distanceY = -tag.getLeftRightDistance();
       outputY = controller.calculate(distanceY);

       distanceX = tag.getForwardReverseDistance();
       errorX = distanceX - TARGET_Z;
       outputX = controller.calculate(errorX);

       if (outputX == 0 && outputY == 0) {
           swerveDrive.stop();
           done = true;
           return;
       }

       swerveDrive.drive(new ChassisSpeeds(-outputX, -outputY, 0.0));
       done = false;
    }

    public boolean isFinished() {
        return done;
    }
}
