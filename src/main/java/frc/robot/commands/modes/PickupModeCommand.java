package frc.robot.commands.modes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmPresetCommand;

/**
 * Enters pickup mode for picking up from the floor:
 *   - robot relative is true
 *   - turbo mode is off
 *   - orbit mode is off
 *   - arm to pickup preset
 *
 * This command isn't over until all those actions have completed.
 */
public class PickupModeCommand extends SequentialCommandGroup {

    public PickupModeCommand(Robot robot) {
        addCommands(
            new InstantCommand(() -> robot.swerveDrive.setTurboMode(false)),
            new InstantCommand(() -> robot.swerveDrive.setOrbitMode(false)),
            new InstantCommand(() -> robot.swerveDrive.setRobotRelative(false)),
            new ArmPresetCommand(robot.arm, ArmPresetCommand.PICKUP_POSITION));
    }
}

    