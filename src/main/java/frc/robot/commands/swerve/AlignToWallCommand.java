package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.HalfBakedSpeedController;

public class AlignToWallCommand extends CommandBase {

    public static final double MAX_SPEED = Units.degreesToRadians(90);
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
    private boolean done;

    public AlignToWallCommand(SwerveDriveSubsystem drive, Wall wall) {
        this.drive = drive;
        this.wall = wall;
        this.controller = new HalfBakedSpeedController(
                MIN_THRESHOLD, MAX_THRESHOLD,
                MIN_SPEED, MAX_SPEED);
    }

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {

        double current = drive.getYaw().getDegrees();
        double error = wall == Wall.GRID
                ? calculateGridError(current)
                : calculateLoadingStationError(current);
        double output = controller.calculate(error);

        SmartDashboard.putNumber("AlignToWall/Current", current);
        SmartDashboard.putNumber("AlignToWall/Error", error);
        SmartDashboard.putNumber("AlignToWall/Output", output);

        if (output != 0.0) {
            drive.drive(new ChassisSpeeds(0, 0, output));
        } else {
            drive.stop();
            done = true;
        }
    }

    private static double calculateLoadingStationError(double current) {
        return current > 0
                ? 180 - current
                : 180 + current;
    }

    private static double calculateGridError(double current) {
        return -current;
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
