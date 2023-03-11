package frc.robot;

import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.SingleJoystickMapping;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.commands.swerve.AlignToAprilTagCommand;
import frc.robot.commands.swerve.AlignToWallCommand;
import frc.robot.commands.swerve.SwerveCommands;
import frc.robot.commands.swerve.SwerveTeleopCommand;

import frc.robot.commands.swerve.SwerveTeleopCommand;



import java.util.function.BooleanSupplier;

public class PIDControlMapping {

    private final Robot robot;
    private final SwerveDriveSubsystem drive;

    private final CommandXboxController driver;

    public PIDControlMapping(Robot robot, CommandXboxController driver){

        this.robot = robot;
        this.drive = robot.swerveDrive;

        this.driver = driver;

        mapDriver();

    }

    private void mapDriver(){

    SingleJoystickMapping stick = new SingleJoystickMapping(
            driver);

    // turbo if the driver wants it
    BooleanSupplier turboSupplier = () -> driver.getLeftTriggerAxis() > 0.5;

    // sniper if the driver wants it OR ops has taken over
    BooleanSupplier sniperSupplier = ()
            -> driver.getHID().getLeftBumper();

        drive.setDefaultCommand(new SwerveTeleopCommand(
                drive,
                stick::getForwardReverse,
                stick::getStrafeLeftRight,
                stick::getRotateLeftRight,
                turboSupplier,
                sniperSupplier));

                driver.start().onTrue(SwerveCommands.zeroGyro(drive));
                driver.leftStick().onTrue(SwerveCommands.turnWheels(drive, 90));
                driver.x().onTrue(AlignToWallCommand.grid(drive));
                driver.y().onTrue(AlignToWallCommand.loadingStation(drive));
                driver.back().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
        
                // triggers
                driver.rightTrigger(0.5)
                    .onTrue(SwerveCommands.setOrbitMode(drive, true))
                    .onFalse(SwerveCommands.setOrbitMode(robot.swerveDrive, false));
        
                driver.povLeft().onTrue(new AlignToAprilTagCommand(robot.swerveDrive, robot.vision));

    }



}
