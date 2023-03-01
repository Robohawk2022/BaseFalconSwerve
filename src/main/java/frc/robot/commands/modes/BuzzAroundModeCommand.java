package frc.robot.commands.modes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmPresetCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

/**
 * Enters "buzz around mode":
 *   - robot relative is false
 *   - turbo mode is on
 *   - orbit mode is off
 *   - arm to travel position
 *
 * This command isn't over until all those actions have completed,
 * BUT we don't want to tie up the swerve drive while waiting for
 * the arm to reach its target position.
 */
public class BuzzAroundModeCommand extends SequentialCommandGroup {

    public BuzzAroundModeCommand(Robot robot) {
        addCommands(new InstantCommand(() -> init(robot.swerveDrive)));
        addCommands(new ArmPresetCommand(robot.arm, ArmPresetCommand.TRAVEL_POSITION));
    }

    public void init(SwerveDriveSubsystem drive) {
        drive.setRobotRelative(false);
        drive.setTurboMode(true);
        drive.setOrbitMode(false);
    }
}
