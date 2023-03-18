package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.SwerveUtils;
import frc.robot.util.HalfBakedSpeedController;

public class AlignToWallCommand extends CommandBase {

    public static final double MAX_SPEED = Units.degreesToRadians(120);
    public static final double MIN_SPEED = Units.degreesToRadians(10);
    public static final double MIN_THRESHOLD = 2;
    public static final double MAX_THRESHOLD = 30;

    public enum Wall {
        GRID,
        LOAD;
    }

    private final SwerveDriveSubsystem drive;
    private final Wall wall;
    private final HalfBakedSpeedController controller;
    private double current;
    private double error;
    private double output;
    private boolean done;

    public AlignToWallCommand(SwerveDriveSubsystem drive, Wall wall) {

        this.drive = drive;
        this.wall = wall;
        this.controller = new HalfBakedSpeedController(
                MIN_THRESHOLD, MAX_THRESHOLD,
                MIN_SPEED, MAX_SPEED);

        addRequirements(drive);

        SmartDashboard.putData("AlignToWallCommand-"+wall.name(), builder -> {
            builder.addBooleanProperty("Done", () -> done, null);
            builder.addDoubleProperty("Current", () -> current, null);
            builder.addDoubleProperty("Error", () -> error, null);
            builder.addDoubleProperty("Output", () -> output, null);
        });
    }

    @Override
    public void initialize() {
        done = false;
        current = 0;
        error = 0;
        output = 0;
    }

    @Override
    public void execute() {

        current = drive.getYaw().getDegrees();
        error = SwerveUtils.angleError(current, wall == Wall.GRID ? 0 : 180);
        output = controller.calculate(error);

        if (output != 0.0) {
            drive.drive(new ChassisSpeeds(0, 0, output));
        } else {
            drive.stop();
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    public static AlignToWallCommand grid(SwerveDriveSubsystem drive) {
        return new AlignToWallCommand(drive, Wall.GRID);
    }

    public static AlignToWallCommand loadingStation(SwerveDriveSubsystem drive) {
        return new AlignToWallCommand(drive, Wall.LOAD);
    }
}
