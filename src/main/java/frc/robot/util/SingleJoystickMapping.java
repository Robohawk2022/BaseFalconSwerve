package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.function.BooleanSupplier;

public class SingleJoystickMapping {

    public static final double DEADBAND = 0.1;

    private final CommandXboxController driver;

    public SingleJoystickMapping(CommandXboxController driver) {

        this.driver = driver;
    
    }
    private double clean(double value) {
        value = MathUtil.applyDeadband(value, DEADBAND);
        value = Math.abs(value) * value;
        return value;
    }

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
        double value = driver.getRawAxis(axis);
        return clean(value);
    }

}