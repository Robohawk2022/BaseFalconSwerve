package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

/*
 * Inspired by https://github.com/Team865/FRC-2023/blob/main/src/main/java/ca/warp7/frc2023/commands/BalanceCommand.java
 */
public class PIDParkCommand extends CommandBase {

    public static final double PITCH_TOLERANCE = 10;
    public static final double STARTING_P = 0.035;

    private final SwerveDriveSubsystem drive;
    private final PIDController controller;
    private double pitch;
    private double power;
    private double lastPower;
    private boolean done;

    public PIDParkCommand(SwerveDriveSubsystem drive) {

        this.drive = drive;
        this.controller = new PIDController(STARTING_P, 0, 0, 2);
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
        lastPower = 1;
        controller.setP(STARTING_P);
        pitch = drive.getPitch();
        done = false;
    }

    @Override
    public void execute() {

        if (DriverStation.getMatchTime() < 0.5) {
            drive.stop();
            done = true;
            return;
        }

        pitch = drive.getPitch();

        if (Math.abs(pitch) <= PITCH_TOLERANCE) {
            power = 0;
            done = false;
            drive.stop();
        } else {
            power = -controller.calculate(pitch, 0);
            done = false;
            drive.drive(new ChassisSpeeds(power, 0, 0));
        }

        if (lastPower == 0 && power != 0) {
            controller.setP(controller.getP() * 0.75);
        }
        lastPower = power;
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}