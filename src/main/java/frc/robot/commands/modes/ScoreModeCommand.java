package frc.robot.commands.modes;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmPresetCommand;
import frc.robot.commands.swerve.AlignToAprilTagCommand;
import frc.robot.commands.swerve.AlignToWallCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Enters "drop off" mode for depositing to the grid:
 *   - robot relative is false
 *   - turbo mode is off
 *   - orbit mode is off
 *   - align to wall (0)
 *   - align to april tag
 * This command isn't over until all those actions have completed.
 */
public class ScoreModeCommand extends ParallelCommandGroup {

    public ScoreModeCommand(Robot robot) {
        addCommands(Commands.sequence(
                new InstantCommand(() -> init(robot.swerveDrive)),
                new AlignToWallCommand(robot, 180),
                new AlignToAprilTagCommand(robot.swerveDrive, robot.vision)));
        addCommands(new ArmPresetCommand(robot.arm, ArmPresetCommand.MIDDLE_POSITION));
    }
    
    public void init(SwerveDriveSubsystem drive) {
        drive.setRobotRelative(false);
        drive.setTurboMode(false);
        drive.setOrbitMode(false);
    }
}
