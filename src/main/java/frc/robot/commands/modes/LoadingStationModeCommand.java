package frc.robot.commands.modes;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmPresetCommand;
import frc.robot.commands.swerve.AlignToAprilTagCommand;
import frc.robot.commands.swerve.AlignToWallCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

/**
 * Enters "loading" mode for grabbing from the loading station
 *   - robot relative is false
 *   - turbo mode is off
 *   - orbit mode is off
 *   - align to wall (180)
 *   - align to april tag
 *   - arm to loading preset
 *
 * This command isn't over until all those actions have completed,
 * BUT we can do the arm thing and the driving things in parallel.
 */
public class LoadingStationModeCommand extends ParallelCommandGroup {

    public LoadingStationModeCommand(Robot robot) {
        addCommands(Commands.sequence(
            new InstantCommand(() -> init(robot.swerveDrive)),
            new AlignToWallCommand(robot, 180),
            new AlignToAprilTagCommand(robot.swerveDrive, robot.vision)));
        addCommands(new ArmPresetCommand(robot.arm, ArmPresetCommand.LOAD_POSITION));
    }
    
    public void init(SwerveDriveSubsystem drive) {
        drive.setRobotRelative(false);
        drive.setTurboMode(false);
        drive.setOrbitMode(false);
    }
}
