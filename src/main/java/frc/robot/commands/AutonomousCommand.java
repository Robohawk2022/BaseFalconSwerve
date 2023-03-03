package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmPresetCommand;
import frc.robot.commands.swerve.SwerveFixedSpeedCommand;

/**
 * This is where all the logic for our autonomous program will go.
 */
public class AutonomousCommand extends SequentialCommandGroup {

    public static final ChassisSpeeds BACKUP_SPEED = new ChassisSpeeds(-Units.inchesToMeters(50), 0, 0);

    public AutonomousCommand(Robot robot) {
        addCommands(new ArmPresetCommand(robot.arm, ArmPresetCommand.HIGH_POSITION));
        addCommands(HandCommands.release(robot.hand));
        addCommands(new WaitCommand(0.5));
        addCommands(new SwerveFixedSpeedCommand(robot.swerveDrive, BACKUP_SPEED, false, 3.0));
    }
}
