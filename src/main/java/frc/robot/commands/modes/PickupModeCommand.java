package frc.robot.commands.modes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmPresetCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

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
        addCommands(new InstantCommand(() -> init(robot.swerveDrive)));
        addCommands(new ArmPresetCommand(robot.arm, ArmPresetCommand.PICKUP_POSITION));
    }
    
    public void init(SwerveDriveSubsystem drive) {
        drive.setRobotRelative(true);
        drive.setTurboMode(false);
        drive.setOrbitMode(false);
    }
}

    