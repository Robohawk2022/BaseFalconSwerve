package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

/**
 * Inspired by https://github.com/iron-claw-972/FRC2023/blob/main/src/main/java/frc/robot/commands/BalanceCommand.java
 */
public class IntervalParkCommand extends CommandBase {

    private final SwerveDriveSubsystem drive;

    public IntervalParkCommand(SwerveDriveSubsystem drive) {

        this.drive = drive;

        SmartDashboard.putData("IntervalParkCommand", builder -> {

        });

        addRequirements(drive);
    }
}
