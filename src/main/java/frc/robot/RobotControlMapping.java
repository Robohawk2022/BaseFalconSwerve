package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.arm.ArmCommands;
import frc.robot.commands.swerve.AlignToAprilTagCommand;
import frc.robot.commands.swerve.SwerveCommands;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.commands.HandCommands;
import frc.robot.commands.arm.ArmCalibrationCommand;
import frc.robot.commands.arm.ArmTeleopCommand;
import frc.robot.commands.modes.BuzzAroundModeCommand;
import frc.robot.commands.modes.DropOffModeCommand;
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
        driver.x().onTrue(new BuzzAroundModeCommand(robot));
        driver.y().onTrue(new PickupModeCommand(robot));
        driver.a().onTrue(new LoadingStationModeCommand(robot));
        driver.b().onTrue(new DropOffModeCommand(robot));
        driver.start().onTrue(SwerveCommands.zeroGyro(drive));
        driver.leftStick().onTrue(SwerveCommands.turnWheels(drive, 90));

        // triggers
        driver.rightTrigger(0.5)
            .onTrue(SwerveCommands.setOrbitMode(drive, true))
            .onFalse(SwerveCommands.setOrbitMode(robot.swerveDrive, false));
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
        ops.x().onTrue(ArmCommands.extendBrake(arm));
        ops.y().onTrue(ArmCommands.retractBrake(arm));
        ops.start().onTrue(new ArmCalibrationCommand(arm));

        // bumpers
        ops.leftBumper().onTrue(SwerveCommands.hopLeft(drive, 22.0));
        ops.rightBumper().onTrue(SwerveCommands.hopRight(drive, 22.0));

    }
}
