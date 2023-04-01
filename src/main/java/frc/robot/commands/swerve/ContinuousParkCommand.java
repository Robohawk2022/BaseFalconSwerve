package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.HalfBakedSpeedController;

public class ContinuousParkCommand extends CommandBase {

    private final SwerveDriveSubsystem drive;
    private final HalfBakedSpeedController controller;
    private double currentPitch;
    private double currentOutput;

    public ContinuousParkCommand(SwerveDriveSubsystem drive) {

        this.drive = drive;
        this.controller = new HalfBakedSpeedController(5, 15, 0.1, 0.35);
        this.currentPitch = drive.getPitch();
        this.currentOutput = 0;

        SmartDashboard.putData("ParkCommand", builder -> {
            builder.addDoubleProperty("Output", () -> currentOutput, null);
            builder.addDoubleProperty("Pitch", () -> currentPitch, null);
        });

        addRequirements(drive);
    }

    public void initialize() {
        currentPitch = drive.getPitch();
        currentOutput = 0;
    }

    public void execute() {
        currentPitch = drive.getPitch();
        currentOutput = controller.calculate(currentPitch);
        drive.drive(new ChassisSpeeds(currentOutput, 0, 0));
    }}