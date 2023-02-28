package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmCalibrationCommand;
import frc.robot.commands.arm.ArmPresetCommand;
import frc.robot.commands.swerve.AlignToAprilTagCommand;
import frc.robot.commands.swerve.ParkingOnThePlatformCommand;
import frc.robot.commands.swerve.RelativeTrajectoryCommand;
import frc.robot.commands.swerve.SwerveCommands;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

/**
 * This is where all the logic for our autonomous program will go.
 */
public class AutonomousCommand extends SequentialCommandGroup {

    public static final double MAX_SPEED_MPS = Units.feetToMeters(3.0);

    public AutonomousCommand(Robot robot) {

        // in parallel, we will grip whatever we're holding and zero the gyro for future reference
        addCommands(Commands.parallel(
                SwerveCommands.zeroGyro(robot.swerveDrive),
                HandCommands.grab(robot.hand)));

        // after we are calibrated and gripping whatever we have, let's calibrate the arm;
        // at the same time, we can drive for our first destination
        addCommands(Commands.parallel(
                new ArmCalibrationCommand(robot.arm),
                generatePathToGrid(robot.swerveDrive)));

// TODO
//        addCommands(Commands.parallel(
//                new ArmPresetCommand(robot.arm, ArmPresetCommand.HIGH_POSITION),
//                new AlignToAprilTagCommand(robot.swerveDrive, robot.vision)));

        // score!
        addCommands(HandCommands.release(robot.hand));

        // reset the arm and head towards the opposite side of the charging station
// TODO
//        addCommands(Commands.parallel(
//                new ArmPresetCommand(robot.arm, ArmPresetCommand.BALANCE_POSITION),
//                generatePathToChargingStation(robot.swerveDrive)));

        // mount the platform and then park it until the end of autonomous
// TODO
//        addCommands(new ParkingOnThePlatformCommand(robot, null));
//        addCommands(SwerveCommands.turnWheels(robot.swerveDrive, 90));
    }

    protected Command generatePathToGrid(SwerveDriveSubsystem drive) {
        // TODO where do we go from here?
        return RelativeTrajectoryCommand.makeCommand(drive, MAX_SPEED_MPS,
                new Translation2d(1, 0));
    }

    protected Command generatePathToChargingStation(SwerveDriveSubsystem drive) {
        // TODO where do we go from here?
        return RelativeTrajectoryCommand.makeCommand(drive, MAX_SPEED_MPS,
                new Translation2d(-1, 0));
    }
}
