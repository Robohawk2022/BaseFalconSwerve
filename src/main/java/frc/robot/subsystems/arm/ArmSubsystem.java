package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.arm.ArmConfig.*;

public class ArmSubsystem extends SubsystemBase {

    private final CANSparkMax rotateMotor;
    private final RelativeEncoder rotateEncoder;
    private final DigitalInput rotateLimit;
    private double rotateMin;
    private double rotateMax;

    private final CANSparkMax extendMotor;
    private final RelativeEncoder extendEncoder;
    private final DigitalInput extendLimit;
    private final Solenoid extendBrake;
    private double extendMin;
    private double extendMax;

    public ArmSubsystem() {

        rotateMotor = new CANSparkMax(ROTATION_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
        rotateMotor.restoreFactoryDefaults();
        rotateMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rotateMotor.setOpenLoopRampRate(ROTATOR_RAMP_RATE);
        rotateMotor.setClosedLoopRampRate(ROTATOR_RAMP_RATE);
        rotateMotor.setInverted(ROTATION_INVERTED);
        rotateMotor.setSmartCurrentLimit(ROTATION_MAX_CURRENT);
        rotateEncoder = rotateMotor.getEncoder();
        rotateEncoder.setPositionConversionFactor(ROTATION_FACTOR);
        rotateLimit = new DigitalInput(ROTATION_LIMIT_ID);

        extendMotor = new CANSparkMax(EXTENSION_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
        extendMotor.restoreFactoryDefaults();
        extendMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        extendMotor.setOpenLoopRampRate(EXTENDER_RAMP_RATE);
        extendMotor.setClosedLoopRampRate(EXTENDER_RAMP_RATE);
        extendMotor.setInverted(EXTENSION_INVERTED);
        extendMotor.setSmartCurrentLimit(EXTENSION_MAX_CURRENT);
        extendEncoder = extendMotor.getEncoder();
        extendEncoder.setPositionConversionFactor(EXTENSION_FACTOR);
        extendLimit = new DigitalInput(EXTENSION_LIMIT_ID);

        extendBrake = null; // new Solenoid(5, PneumaticsModuleType.REVPH, EXTENSION_BRAKE_CHANNEL);
            
        clearLimits();

        SmartDashboard.putData("Rotator", builder -> {
            builder.addDoubleProperty("Current", this::getAngle, null);
            builder.addBooleanProperty("Limit", this::atRotateLimit, null);
            builder.addDoubleProperty("Max", () -> rotateMax, null);
            builder.addDoubleProperty("Min", () -> rotateMin, null);
        });
        SmartDashboard.putData("Extender", builder -> {
            builder.addDoubleProperty("Current", this::getLength, null);
            builder.addBooleanProperty("Limit", this::atExtendLimit, null);
            builder.addDoubleProperty("Max", () -> extendMax, null);
            builder.addDoubleProperty("Min", () -> extendMin, null);
        });
    }

    public boolean atExtendLimit() {
        return extendLimit.get() == EXTENSION_LIMIT_PRESSED;
    }

    public boolean atRotateLimit() {
        return rotateLimit.get() == ROTATION_LIMIT_PRESSED;
    }
    
    public double getAngle() {
        return rotateEncoder.getPosition();
    }

    public double getLength() {
        return extendEncoder.getPosition();
    }

    public void clearLimits() {
        rotateMin = extendMin = Double.NEGATIVE_INFINITY;
        rotateMax = extendMax = Double.POSITIVE_INFINITY;
    }

    public void retractParkingBrake(){
        extendBrake.set(true);
    }

    public boolean calibrationComplete() {

        double rotateOutput = 0.0;
        if (rotateMin == Double.MIN_VALUE) {
            if (atRotateLimit()) {
                rotateMax = rotateEncoder.getPosition() - ROTATE_TRAVEL_BUFFER;
                rotateMin = rotateMax - ROTATE_PHYSICAL_MAX + 2 * ROTATE_TRAVEL_BUFFER;
            } else {
                rotateOutput = ROTATOR_MIN_SPEED;
            }
        }
        rotateMotor.set(rotateOutput);

        double extendOutput = 0.0;
        if (extendMin == Double.MIN_VALUE) {
            if (atExtendLimit()) {
                extendMin = extendEncoder.getPosition() + EXTENDER_TRAVEL_BUFFER;
                extendMax = extendMin + EXTENDER_PHYSICAL_MAX - 2 * EXTENDER_TRAVEL_BUFFER;
            } else {
                extendOutput = -EXTENDER_MAX_SPEED;
            }
        }
        extendMotor.set(extendOutput);

        return rotateOutput == 0.0 && extendOutput == 0.0;
    }

    public void extendAt(double percentOutput) {
        if (percentOutput > 0 && getLength() > extendMax) {
            percentOutput = 0;
        }
        if (percentOutput < 0 && getLength() < extendMin) {
            percentOutput = 0;
        }
        percentOutput = MathUtil.clamp(percentOutput, -EXTENDER_MAX_SPEED, EXTENDER_MAX_SPEED);
        extendMotor.set(percentOutput);
    }

    public void rotateAt(double percentOutput) {
        if (percentOutput > 0 && getAngle() > rotateMax) {
            percentOutput = 0;
        }
        if (percentOutput < 0 && getAngle() < rotateMin) {
            percentOutput = 0;
        }
        percentOutput = MathUtil.clamp(percentOutput, -ROTATOR_MAX_SPEED, ROTATOR_MAX_SPEED);
        rotateMotor.set(percentOutput);
    }
}
