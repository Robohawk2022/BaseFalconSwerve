package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

/*
 * Inspired by https://github.com/Team865/FRC-2023/blob/main/src/main/java/ca/warp7/frc2023/commands/BalanceCommand.java
 */
public class PIDParkCommand extends CommandBase {

    private final SwerveDriveSubsystem drive;
    private final PIDController controller;
    private double pitch;
    private double power;
    private boolean done;

    public PIDParkCommand(SwerveDriveSubsystem drive) {

        this.drive = drive;
        this.controller = new PIDController(0.043, 0, 0, 2);
        this.pitch = drive.getPitch();
        this.power = 0;
        this.done = false;

        SmartDashboard.putData("PIDParkCommand", builder -> {
            builder.addBooleanProperty("Done", () -> done, null);
            builder.addDoubleProperty("Pitch", () -> pitch, null);
            builder.addDoubleProperty("Power", () -> power, null);
        });

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        power = 0;
        pitch = drive.getPitch();
        done = false;
    }

    @Override
    public void execute() {

        pitch = drive.getPitch();

        if (Math.abs(pitch) <= controller.getPositionTolerance()) {
            power = 0;
            done = true;
            drive.stop();
        } else {
            power = controller.calculate(pitch, 0);
            done = false;
            drive.drive(new ChassisSpeeds(power, 0, 0));
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}