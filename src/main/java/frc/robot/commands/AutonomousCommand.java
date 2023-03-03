package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
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
        addCommands(
                log("- auto 1 -"),
                robot.arm.toPreset(ArmPresetCommand.HIGH_POSITION),
                log("- auto 2 -"),
                robot.hand.releaseCommand(),
                log("- auto 3 -"),
                Commands.waitSeconds(0.5),
                log("- auto 4 -"),
                Commands.parallel(
                        robot.arm.toPreset(ArmPresetCommand.TRAVEL_POSITION),
                        Commands.sequence(
                            new SwerveFixedSpeedCommand(robot.swerveDrive, BACKUP_SPEED, false, 3.0),
                            AlignToWallCommand.loadingStation(robot.swerveDrive),
                            new SwerveFixedSpeedCommand(robot.swerveDrive, JUKE_SPEED, false, 3.0)
                        )
                )
        );
    }

    public static Command log(String message) {
        return new InstantCommand(() -> System.err.println(message));
    }
}
