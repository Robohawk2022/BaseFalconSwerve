package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmPresetCommand;
import frc.robot.commands.swerve.AlignToWallCommand;
import frc.robot.commands.swerve.SwerveFixedSpeedCommand;

/**
 * This is where all the logic for our autonomous program will go.
 */
public class AutonomousCommand extends SequentialCommandGroup {

    public static enum Program {
        NONE,
        DROP_ONLY,
        DROP_EXIT,
        DROP_EXIT_MOUNT_R,
        DROP_EXIT_MOUNT_L
    }

    public static final ChassisSpeeds JUKE_SPEED = new ChassisSpeeds(Units.inchesToMeters(0), Units.inchesToMeters(6), -Units.degreesToRadians(0));
    public static final ChassisSpeeds BACKUP_SPEED = new ChassisSpeeds(Units.inchesToMeters(80), Units.inchesToMeters(0), -Units.degreesToRadians(-8));

    public static Command generateProgram(Robot robot, Program which) {

        SequentialCommandGroup group = new SequentialCommandGroup();

        // no program - just sit still
        if (which == Program.NONE) {
            return group;
        }

        // all other commands include at least the drop
        group.addCommands(
                robot.arm.toPreset(ArmPresetCommand.HIGH_POSITION),
                robot.hand.releaseCommand()
        );
        if (which == Program.DROP_ONLY) {
            return group;
        }

        // if we're exiting the community area, we raise our arm and go
        group.addCommands(Commands.parallel(
                robot.arm.toPreset(ArmPresetCommand.TRAVEL_POSITION),
                new SwerveFixedSpeedCommand(robot.swerveDrive, BACKUP_SPEED, false, 3.0)));
        if (which == Program.DROP_EXIT) {
            return group;
        }

        // if we're mounting, the question is left or right
        ChassisSpeeds jukeSpeed = which == Program.DROP_EXIT_MOUNT_L
                ? new ChassisSpeeds(0, Units.inchesToMeters(22), 0)
                : new ChassisSpeeds(0, Units.inchesToMeters(-22), 0);
        group.addCommands(
                new SwerveFixedSpeedCommand(robot.swerveDrive, jukeSpeed, false, 1.0),
                AlignToWallCommand.loadingStation(robot.swerveDrive));
        return group;
    }
}
