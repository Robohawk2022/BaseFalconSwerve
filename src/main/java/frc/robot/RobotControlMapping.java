package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.arm.ArmPresetCommand;
import frc.robot.commands.swerve.AlignToWallCommand;
import frc.robot.commands.swerve.RelativeTrajectoryCommand;
import frc.robot.commands.swerve.SwerveCommands;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.commands.HandCommands;
import frc.robot.commands.arm.ArmCalibrationCommand;
import frc.robot.commands.arm.ArmTeleopCommand;
import frc.robot.commands.modes.BuzzAroundModeCommand;
import frc.robot.commands.modes.ScoreModeCommand;
import frc.robot.commands.modes.LoadingStationModeCommand;
import frc.robot.commands.modes.PickupModeCommand;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.MultiJoystickMapping;

/**
 * All of the mapping of controls to commands for the production robot happens here.
 */
public class RobotControlMapping {

    private final Robot robot;
    private final SwerveDriveSubsystem drive;
    private final HandSubsystem hand;
    private final ArmSubsystem arm;
    private final VisionSubsystem vision;
    private final CommandXboxController driver;
    private final CommandXboxController ops;

    public RobotControlMapping(Robot robot, CommandXboxController driver, CommandXboxController ops) {

        this.robot = robot;
        this.drive = robot.swerveDrive;
        this.hand = robot.hand;
        this.arm = robot.arm;
        this.vision = robot.vision;

        this.driver = driver;
        this.ops = ops;

        mapSharedSticks();
        mapDriver();
        mapOps();
    }

    private void mapSharedSticks() {

        // ops switches to driving if they pull their left trigger
        MultiJoystickMapping sticks = new MultiJoystickMapping(
                driver,
                ops,
                () -> ops.getRightTriggerAxis() > 0.5);

        drive.setDefaultCommand(new SwerveTeleopCommand(
                drive,
                sticks::getForwardReverse,
                sticks::getStrafeLeftRight,
                sticks::getRotateLeftRight));

        arm.setDefaultCommand(new ArmTeleopCommand(
                arm,
                sticks::getArmRotate,
                sticks::getArmExtend));
    }

    /*
██████╗ ██████╗ ██╗██╗   ██╗███████╗██████╗
██╔══██╗██╔══██╗██║██║   ██║██╔════╝██╔══██╗
██║  ██║██████╔╝██║██║   ██║█████╗  ██████╔╝
██║  ██║██╔══██╗██║╚██╗ ██╔╝██╔══╝  ██╔══██╗
██████╔╝██║  ██║██║ ╚████╔╝ ███████╗██║  ██║
╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═══╝  ╚══════╝╚═╝  ╚═╝
     */

    private void mapDriver() {

        // buttons
        driver.start().onTrue(SwerveCommands.zeroGyro(drive));
        driver.leftStick().onTrue(SwerveCommands.turnWheels(drive, 90));
        driver.x().onTrue(SwerveCommands.scootForward(drive, 6.0));
        driver.b().onTrue(HandCommands.grab(hand));
        driver.a().onTrue(HandCommands.release(hand));
        driver.back().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

        // triggers
        driver.rightTrigger(0.5)
            .onTrue(SwerveCommands.setOrbitMode(drive, true))
            .onFalse(SwerveCommands.setOrbitMode(robot.swerveDrive, false));
        driver.leftTrigger(0.5)
            .onTrue(SwerveCommands.setTurboMode(drive, true))
            .onFalse(SwerveCommands.setTurboMode(robot.swerveDrive, false));

        // POV mapping
        driver.povUp().onTrue(new BuzzAroundModeCommand(robot));
        driver.povRight().onTrue(new PickupModeCommand(robot));
        driver.povLeft().onTrue(new LoadingStationModeCommand(robot));
        driver.povDown().onTrue(new ScoreModeCommand(robot));

        // testing out a trajectory
        driver.rightBumper()
            .onTrue(RelativeTrajectoryCommand.makeCommand(drive,
                Units.feetToMeters(4.0),
                new Translation2d(1.0, 1.0),
                new Translation2d(0.0, -2.0),
                new Translation2d(-1.0, -1.0)));
    }

    /*
 ██████╗ ██████╗ ███████╗
██╔═══██╗██╔══██╗██╔════╝
██║   ██║██████╔╝███████╗
██║   ██║██╔═══╝ ╚════██║
╚██████╔╝██║     ███████║
 ╚═════╝ ╚═╝     ╚══════╝
     */

    private void mapOps() {

        // buttons
        ops.b().onTrue(HandCommands.grab(hand));
        ops.a().onTrue(HandCommands.release(hand));
        ops.x().onTrue(new AlignToWallCommand(drive, AlignToWallCommand.Wall.GRID));
        ops.y().onTrue(SwerveCommands.scootForward(drive, 6.0));
        ops.start().onTrue(new ArmCalibrationCommand(arm));
        driver.back().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

        // bumpers (left and right are reversed b/c robot is facing driver)
        ops.leftBumper().onTrue(SwerveCommands.scootRight(drive, 22.0));
        ops.rightBumper().onTrue(SwerveCommands.scootLeft(drive, 22.0));

        // dpad
        ops.povUp().onTrue(new ArmPresetCommand(arm, ArmPresetCommand.HIGH_POSITION));
        ops.povLeft().onTrue(new ArmPresetCommand(arm, ArmPresetCommand.MIDDLE_POSITION));
        ops.povRight().onTrue(new ArmPresetCommand(arm, ArmPresetCommand.MIDDLE_POSITION));
        ops.povDown().onTrue(new ArmPresetCommand(arm, ArmPresetCommand.PICKUP_POSITION));
    }
}
