package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.arm.ArmPresetCommand;
import frc.robot.commands.swerve.AlignToAprilTagCommand;
import frc.robot.commands.swerve.AlignToWallCommand;
import frc.robot.commands.swerve.SwerveCommands;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.commands.HandCommands;
import frc.robot.commands.arm.ArmCommands;
import frc.robot.commands.arm.ArmTeleopCommand;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.MultiJoystickMapping;

import java.util.function.BooleanSupplier;

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

        // ops switches to driving if they pull their right trigger
        MultiJoystickMapping sticks = new MultiJoystickMapping(
                driver,
                ops,
                () -> ops.getRightTriggerAxis() > 0.5);

        // turbo if the driver wants it
        BooleanSupplier turboSupplier = () -> driver.getLeftTriggerAxis() > 0.5;

        // sniper if the driver wants it OR ops has taken over
        BooleanSupplier sniperSupplier = ()
                -> driver.getHID().getLeftBumper()
                || ops.getRightTriggerAxis() > 0.5;

        drive.setDefaultCommand(new SwerveTeleopCommand(
                drive,
                sticks::getForwardReverse,
                sticks::getStrafeLeftRight,
                sticks::getRotateLeftRight,
                turboSupplier,
                sniperSupplier));

        arm.setDefaultCommand(new ArmTeleopCommand(
                arm,
                sticks::getArmRotate,
                sticks::getArmExtend));
    }

    private void mapDriver() {

        // buttons
        driver.start().onTrue(SwerveCommands.zeroGyro(drive));
        driver.leftStick().onTrue(SwerveCommands.turnWheels(drive, 90));
        // driver.x().onTrue(new ProxyCommand(() -> robot.auto.createCommand(robot)));
        driver.x().onTrue(AlignToWallCommand.grid(drive));
        driver.y().onTrue(AlignToWallCommand.loadingStation(drive));
        driver.b().onTrue(HandCommands.grab(hand));
        driver.a().onTrue(HandCommands.release(hand));
        driver.back().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

        // triggers
        driver.rightTrigger(0.5)
            .onTrue(SwerveCommands.setOrbitMode(drive, true))
            .onFalse(SwerveCommands.setOrbitMode(robot.swerveDrive, false));

        // POV mapping
        driver.povUp().onTrue(ArmCommands.safePreset(arm, ArmPresetCommand.TRAVEL_POSITION));
        driver.povLeft().onTrue(new AlignToAprilTagCommand(robot.swerveDrive, robot.vision));
    }

    private void mapOps() {

        // buttons
        ops.b().onTrue(HandCommands.grab(hand));
        ops.a().onTrue(HandCommands.release(hand));
        ops.x().onTrue(AlignToWallCommand.grid(drive));
        ops.y().onTrue(AlignToWallCommand.loadingStation(drive));
        ops.start().onTrue(ArmCommands.safeCalibrate(arm));
        driver.back().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

        // bumpers (left and right are reversed b/c robot is facing driver)
        ops.leftBumper().onTrue(SwerveCommands.scootRight(drive, 24.0));
        ops.rightBumper().onTrue(SwerveCommands.scootLeft(drive, 24.0));

        // dpad
        ops.povUp().onTrue(ArmCommands.safePreset(arm, ArmPresetCommand.HIGH_POSITION));
        ops.povLeft().onTrue(ArmCommands.safePreset(arm, ArmPresetCommand.MIDDLE_POSITION));
        ops.povRight().onTrue(ArmCommands.safePreset(arm, ArmPresetCommand.MIDDLE_POSITION));
        ops.povDown().onTrue(ArmCommands.safePreset(arm, ArmPresetCommand.PICKUP_POSITION));
    }
}
