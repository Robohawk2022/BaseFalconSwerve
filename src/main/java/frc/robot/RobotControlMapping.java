package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swerve.TurboModeCommand;
import frc.robot.commands.swerve.SetRobotRelativeCommand;
import frc.robot.commands.swerve.SwerveFixedSpeedCommand;
import frc.robot.commands.swerve.SwerveOrbitCommand;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.commands.swerve.ZeroGyroCommand;
import frc.robot.commands.swerve.AlignToAprilTagCommand;
import frc.robot.commands.swerve.ExampleTrajectoryCommand;
import frc.robot.commands.swerve.RotatingWheelsToADegreeCommand;

import static edu.wpi.first.wpilibj.XboxController.Button.kB;
import static edu.wpi.first.wpilibj.XboxController.Button.kLeftBumper;
import static edu.wpi.first.wpilibj.XboxController.Button.kRightBumper;
import static edu.wpi.first.wpilibj.XboxController.Button.kX;
import static edu.wpi.first.wpilibj.XboxController.Button.kY;
import static edu.wpi.first.wpilibj.XboxController.Button.kStart;

/**
 * All of the mapping of controls to commands for the production robot happens here.
 */
public class RobotControlMapping {

    public static final double DEADBAND = 0.1;

    public static double clean(double input) {
        return MathUtil.applyDeadband(input * Math.abs(input), DEADBAND);
    }

    /**
     * Maps additional controls on the driver's joystick
     *    - X will trigger Kyle & Christopher's command
     *    - Y will trigger Ed's command
     *    - Start will zero the gyro again
     * 
     * Add additional button/axis mappings here.
     */
    public static void mapDriverControls(Robot robot, XboxController driverController) {

        /*
         *   - left Y is drive forward/backwards
         *   - left X is strafe left/right
         *   - right X is rotate left/right
         */
        robot.swerveDrive.setDefaultCommand(new SwerveTeleopCommand(
                robot.swerveDrive,
                () -> clean(-driverController.getLeftY()),
                () -> clean(-driverController.getLeftX()),
                () -> clean(-driverController.getRightX())));

        // trigger(driverController, kX, new KyleAndChristopherCommand(robot.swerveDrive));
        trigger(driverController, kY, SwerveFixedSpeedCommand.buildMultiStepProgram(robot.swerveDrive));
        trigger(driverController, kStart, new ZeroGyroCommand(robot.swerveDrive));
        //trigger(driverController, kB, new AlignToWallCommand(robot, 0));
        // trigger(driverController, kB, new MountingToChargeStationIntegratedCommand(robot, driverController));
        try {
            trigger(driverController, kB, new ExampleTrajectoryCommand(robot.swerveDrive));
        } catch (Exception e) {
            e.printStackTrace();
        }
        trigger(driverController, kX, new AlignToAprilTagCommand(robot.swerveDrive, robot.vision));
        trigger(driverController, Button.kLeftStick, new RotatingWheelsToADegreeCommand(robot, 90));
        // trigger(driverController, Button.kBack, new AutonomousCommand(robot));

        // hold the left bumper to run in robot relative mode
        new JoystickButton(driverController, kLeftBumper.value)
                .onTrue(new SetRobotRelativeCommand(robot.swerveDrive, true))
                .onFalse(new SetRobotRelativeCommand(robot.swerveDrive, false));

        // hold the right bumper to double the maximum speed
        new JoystickButton(driverController, kRightBumper.value)
                .onTrue(new TurboModeCommand(robot.swerveDrive, true))
                .onFalse(new TurboModeCommand(robot.swerveDrive, false));

        new Trigger(() -> driverController.getRightTriggerAxis() > 0.5)
                .onTrue(new SwerveOrbitCommand(robot.swerveDrive, true))
                .onFalse(new SwerveOrbitCommand(robot.swerveDrive, false));
    }

    /**
     * Maps controls on the special ops joystick.
     * 
     * Add additional button/axis mappings here.
     */
    public static void mapSpecialOpsControls(Robot robot, XboxController specialOpsController) {
        // trigger(specialOpsController, kY, new ExampleCommand(robot));
    }

    /**
     * Use this to make a specific button trigger a command
     */
    public static void trigger(XboxController controller, Button button, Command command) {
        new JoystickButton(controller, button.value).onTrue(command);
    }
}
