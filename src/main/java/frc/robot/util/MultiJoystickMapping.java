package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.function.BooleanSupplier;

public class MultiJoystickMapping {

    public static final double DEADBAND = 0.1;

    private final CommandXboxController driver;
    private final CommandXboxController ops;
    private final BooleanSupplier override;

    public MultiJoystickMapping(CommandXboxController driver, CommandXboxController ops, BooleanSupplier override) {
        this.driver = driver;
        this.ops = ops;
        this.override = override;
    }

    // we always "clean" joystick inputs by applying a deadband and squaring them
    private double clean(double value) {
        value = MathUtil.applyDeadband(value, DEADBAND);
        value = Math.abs(value) * value;
        return value;
    }

    /*
██████╗ ██████╗ ██╗██╗   ██╗██╗███╗   ██╗ ██████╗
██╔══██╗██╔══██╗██║██║   ██║██║████╗  ██║██╔════╝
██║  ██║██████╔╝██║██║   ██║██║██╔██╗ ██║██║  ███╗
██║  ██║██╔══██╗██║╚██╗ ██╔╝██║██║╚██╗██║██║   ██║
██████╔╝██║  ██║██║ ╚████╔╝ ██║██║ ╚████║╚██████╔╝
╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═══╝  ╚═╝╚═╝  ╚═══╝ ╚═════╝
    */

    // left Y = forward/reverse
    public double getForwardReverse() {
        return -getDriveAxis(XboxController.Axis.kLeftY.value);
    }

    // left X = strafe left/right
    public double getStrafeLeftRight() {
        return -getDriveAxis(XboxController.Axis.kLeftX.value);
    }

    // right X = rotate left/right
    public double getRotateLeftRight() {
        return getDriveAxis(XboxController.Axis.kRightX.value);
    }

    private double getDriveAxis(int axis) {
        double value = override.getAsBoolean() ? ops.getRawAxis(axis) : driver.getRawAxis(axis);
        return clean(value);
    }

    /*
 █████╗ ██████╗ ███╗   ███╗
██╔══██╗██╔══██╗████╗ ████║
███████║██████╔╝██╔████╔██║
██╔══██║██╔══██╗██║╚██╔╝██║
██║  ██║██║  ██║██║ ╚═╝ ██║
╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝
     */

    public double getArmRotate() {
        return -getArmAxis(XboxController.Axis.kLeftY.value);
    }

    public double getArmExtend() {
        return -getArmAxis(XboxController.Axis.kRightY.value);
    }

    private double getArmAxis(int axis) {
        double value = override.getAsBoolean() ? 0 : ops.getRawAxis(axis);
        return clean(value);
    }
}
