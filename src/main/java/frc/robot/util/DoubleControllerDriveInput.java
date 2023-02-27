package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * We may need to accept drive input from both controllers. This wraps the two
 * controllers and implements an "override" - if the driver is supplying input,
 * the input from the ops controller will be ignored.
 */
public class DoubleControllerDriveInput {

    public static final double DEADBAND = 0.1;

    private final CommandXboxController driver;
    private final CommandXboxController specialOps;

    public DoubleControllerDriveInput(CommandXboxController driver, CommandXboxController specialOps) {
        this.driver = driver;
        this.specialOps = specialOps;
    }

    private boolean driverIsSupplyingInputs() {

        double dlx = MathUtil.applyDeadband(driver.getLeftX(), DEADBAND);
        if (dlx != 0) {
            return true;
        }

        double dly = MathUtil.applyDeadband(driver.getLeftX(), DEADBAND);
        if (dly != 0) {
            return true;
        }

        double drx = MathUtil.applyDeadband(driver.getLeftX(), DEADBAND);
        if (drx != 0) {
            return true;
        }

        return false;
    }

    private double clean(double value) {
        value = MathUtil.applyDeadband(value, DEADBAND);
        value = Math.abs(value) * value;
        return value;
    }

    public double getDriveForwardReverse() {
        return driverIsSupplyingInputs()
                ? clean(driver.getLeftY())
                : clean(specialOps.getLeftY());
    }

    public double getDriveStrafeLeftRight() {
        return driverIsSupplyingInputs()
                ? clean(driver.getLeftY())
                : clean(specialOps.getLeftY());
    }

    public double getDriveRotateLeftRight() {
        return driverIsSupplyingInputs()
                ? clean(driver.getLeftY())
                : clean(specialOps.getLeftY());
    }
}
