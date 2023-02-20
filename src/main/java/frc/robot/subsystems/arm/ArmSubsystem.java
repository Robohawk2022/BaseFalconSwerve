package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    public static final double MAX_ROTATE = 0.8;
    public static final double MAX_EXTEND = 0.8;

    public static final int ROTATION_CANID = 1;
    public static final int ROTATION_LIMIT_ID = 1;
    public static final boolean ROTATION_INVERTED = false;
    public static final double ROTATION_TRAVEL_LIMIT = 300;
    public static final double ROTATION_FACTOR = 1.0;

    public static final int EXTENSION_CANID = 1;
    public static final int EXTENSION_LIMIT_ID = 1;
    public static final boolean EXTENSION_INVERTED = false;
    public static final double EXTENSION_TRAVEL_LIMIT = 300;
    public static final double EXTENSION_FACTOR = 1.0;

    private final ArmUnit rotator;
    private final ArmUnit extender;

    public ArmSubsystem() {
        extender = new ArmUnit(EXTENSION_CANID, EXTENSION_LIMIT_ID, EXTENSION_FACTOR, EXTENSION_INVERTED);
        rotator = new ArmUnit(ROTATION_CANID, ROTATION_LIMIT_ID, ROTATION_FACTOR, ROTATION_INVERTED);
        SmartDashboard.putData("Rotator", builder -> {
            builder.addDoubleProperty("Current", () -> rotator.encoder.getPosition(), null);
            builder.addDoubleProperty("Max", () -> rotator.max, null);
            builder.addDoubleProperty("Min", () -> rotator.min, null);
        });
        SmartDashboard.putData("Extender", builder -> {
            builder.addDoubleProperty("Current", () -> extender.encoder.getPosition(), null);
            builder.addDoubleProperty("Max", () -> extender.max, null);
            builder.addDoubleProperty("Min", () -> extender.min, null);
        });
    }

    /**
     * Clears limits on both units
     */
    public void clearLimits() {
        rotator.clearLimits();
        extender.clearLimits();
    }

    /**
     * Calibrates both units
     * @return true when calibration is done on both of them
     */
    public boolean calibrate(double percentRotate, double percentExtend) {
        return rotator.calibrate(percentRotate, ROTATION_TRAVEL_LIMIT)
                && extender.calibrate(percentExtend, EXTENSION_TRAVEL_LIMIT);
    }

    /**
     * @return the current position of the rotator
     */
    public double getRotatorPosition() {
        return rotator.encoder.getPosition();
    }

    /**
     * @return the current position of the extender
     */
    public double getExtenderPosition() {
        return extender.encoder.getPosition();
    }

    /**
     * Moves safely at the specified speed
     */
    public void moveAt(double percentRotate, double percentExtend) {
        rotator.setSafe(percentRotate, MAX_ROTATE);
        extender.setSafe(percentExtend, MAX_EXTEND);
    }

}