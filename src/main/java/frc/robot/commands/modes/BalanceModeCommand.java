package frc.robot.commands.modes;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.Robot;
import frc.robot.commands.arm.ArmPresetCommand;
import frc.robot.commands.swerve.AlignToWallCommand;
import frc.robot.commands.swerve.ParkingOnThePlatformCommand;
import frc.robot.commands.swerve.SwerveCommands;

public class BalanceModeCommand extends ParallelCommandGroup {

    public BalanceModeCommand(Robot robot) {
        addCommands(new ArmPresetCommand(robot.arm, ArmPresetCommand.BALANCE_POSITION));
        addCommands(Commands.sequence(
                new AlignToWallCommand(robot, 0),
                new ParkingOnThePlatformCommand(robot.swerveDrive),
                SwerveCommands.turnWheels(robot.swerveDrive, 90)));
    }
}
