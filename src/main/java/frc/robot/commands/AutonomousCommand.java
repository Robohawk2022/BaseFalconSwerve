package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmPresetCommand;
import frc.robot.commands.swerve.SwerveCommands;
import frc.robot.commands.swerve.SwerveFixedSpeedCommand;
import static frc.robot.subsystems.AutonomusSubystem.*;

/**
 * This is where all the logic for our autonomous program will go.
 */
public class AutonomousCommand extends SequentialCommandGroup {

    
    public static final ChassisSpeeds BACKUP_SPEED = new ChassisSpeeds(
        Units.inchesToMeters(70), 
        Units.inchesToMeters(0), 
        -Units.degreesToRadians(-8));

    public static final ChassisSpeeds JUKE_LEFT = new ChassisSpeeds(
        0, 
        Units.inchesToMeters(60), 
        0);
    
    public static final ChassisSpeeds JUKE_RIGHT = new ChassisSpeeds(
        0, 
        Units.inchesToMeters(60), 
        0);

    public static final ChassisSpeeds MOUNT_SPEED = new ChassisSpeeds(
        Units.feetToMeters(-3.0), 
        Units.inchesToMeters(0), 
        -Units.degreesToRadians(0));

    public static Command generateProgram(Robot robot, String which) {

        SequentialCommandGroup group = new SequentialCommandGroup();

        // no program - just sit still
        if (NONE.equals(which)) {
            return group;
        }

        // all other commands include at least the drop
        group.addCommands(
                robot.arm.toPreset(ArmPresetCommand.MIDDLE_POSITION),
                robot.hand.releaseCommand(),
                Commands.waitSeconds(1)
        );
        if (DROP.equals(which)) {
            return group;
        }

        if(which.equals(DROP_CENTER_EXIT)){
            group.addCommands(Commands.parallel(
                robot.arm.toPreset(ArmPresetCommand.TRAVEL_POSITION),
                new SwerveFixedSpeedCommand(robot.swerveDrive, BACKUP_SPEED, false, 4)));
                return group;
        }

        // if we're exiting the community area, we raise our arm and go
        group.addCommands(Commands.parallel(
                robot.arm.toPreset(ArmPresetCommand.TRAVEL_POSITION),
                new SwerveFixedSpeedCommand(robot.swerveDrive, BACKUP_SPEED, false, 3.0)));
        if (EXIT.equals(which)) {
            return group;
        }

        // if we're mounting, the question is left or right
        ChassisSpeeds jukeSpeed = MOUNT_L.equals(which) ? JUKE_LEFT : JUKE_RIGHT;         
        group.addCommands(
                new SwerveFixedSpeedCommand(robot.swerveDrive, jukeSpeed, false, 1.0),
                new SwerveFixedSpeedCommand(robot.swerveDrive, MOUNT_SPEED, false, 4),
                SwerveCommands.turnWheels(robot.swerveDrive, 90));
        return group;
    }
}
