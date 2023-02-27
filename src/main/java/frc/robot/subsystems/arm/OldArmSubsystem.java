package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.arm.ArmConfig.*;

public class OldArmSubsystem extends SubsystemBase {

    public final ArmUnit rotator;
    public final ArmUnit extender;
    private final Solenoid parkingBreak;

    // TODO MotorControllers: 50 Amphs set current Limit
    // TODO Limit the accelaration of the arm 3s from 0 - 100%
    // We should be conservative on arm testing: arms are rathre fragile
    // TODO Limit the accelaration, velocity graph is better at a triangle
    
    //TODO replace the parkingBreak with actual solenoid
    public OldArmSubsystem() {
        extender = new ArmUnit(EXTENSION_CANID, EXTENSION_LIMIT_ID, EXTENSION_FACTOR, EXTENSION_INVERTED);
        rotator = new ArmUnit(ROTATION_CANID, ROTATION_LIMIT_ID, ROTATION_FACTOR, ROTATION_INVERTED);
        parkingBreak = new Solenoid(PneumaticsModuleType.REVPH, EXTENSION_BRAKE_CHANNEL);
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
    public void retractParkingBreak(){
        parkingBreak.set(true);
    }
    
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

    public void extendAt(double percentOutput) {
        extender.setSafe(percentOutput, EXTENDER_MAX_SPEED);
    }

    public void rotateAt(double percentOutput) {
        rotator.setSafe(percentOutput, ROTATOR_MAX_SPEED);
    }
}
