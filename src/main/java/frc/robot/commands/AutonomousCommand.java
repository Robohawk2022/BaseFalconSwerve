package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmPresetCommand;
import frc.robot.commands.swerve.AlignToWallCommand;
import frc.robot.commands.swerve.SwerveFixedSpeedCommand;

/**
 * This is where all the logic for our autonomous program will go.
 */
public class AutonomousCommand extends SequentialCommandGroup {

    public static final ChassisSpeeds JUKE_SPEED = new ChassisSpeeds(Units.inchesToMeters(0), Units.inchesToMeters(6), -Units.degreesToRadians(0));
    public static final ChassisSpeeds BACKUP_SPEED = new ChassisSpeeds(Units.inchesToMeters(80), Units.inchesToMeters(0), -Units.degreesToRadians(-8));

    public AutonomousCommand(Robot robot) {
        addCommands(new InstantCommand(() -> System.err.println("start")));
        addCommands(new ArmPresetCommand(robot.arm, ArmPresetCommand.HIGH_POSITION));
        addCommands(new InstantCommand(() -> System.err.println("2")));
        addCommands(HandCommands.release(robot.hand));
        addCommands(new InstantCommand(() -> System.err.println("3")));
        addCommands(new WaitCommand(0.5));
        addCommands(Commands.parallel(
            new ArmPresetCommand(robot.arm, ArmPresetCommand.TRAVEL_POSITION),
            Commands.sequence(
                new SwerveFixedSpeedCommand(robot.swerveDrive, BACKUP_SPEED, false, 3.0),
                AlignToWallCommand.loadingStation(robot.swerveDrive),
                new SwerveFixedSpeedCommand(robot.swerveDrive, JUKE_SPEED, false, 3.0)
            )
        ));
    }
}
