package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.arm.ArmPresetCommand;
import frc.robot.commands.arm.ArmCalibrationCommand;

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
    private boolean brakeRetracted;

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

        extendBrake = new Solenoid(5, PneumaticsModuleType.REVPH, EXTENSION_BRAKE_CHANNEL);
        brakeRetracted = false;

        clearLimits();

        SmartDashboard.putData("ArmRotator", builder -> {
            builder.addDoubleProperty("Angle", this::getAngleDelta, null);
            builder.addBooleanProperty("LimitTripped", this::rotateLimitTripped, null);
            builder.addDoubleProperty("EffectiveMax", () -> rotateMax, null);
            builder.addDoubleProperty("EffectiveMin", this::getEffectiveRotateMin, null);
            builder.addDoubleProperty("EncoderMin", () -> rotateMin, null);
            builder.addDoubleProperty("EncoderPosition", this::getAngle, null);
        });
        SmartDashboard.putData("ArmExtender", builder -> {
            builder.addDoubleProperty("Length", this::getLengthDelta, null);
            builder.addBooleanProperty("LimitTripped", this::extendLimitTripped, null);
            builder.addDoubleProperty("EffectiveMax", this::getEffectiveExtendMax, null);
            builder.addDoubleProperty("EffectiveMin", () -> extendMin, null);
            builder.addDoubleProperty("EncoderMax", () -> extendMax, null);
            builder.addDoubleProperty("EncoderPosition", this::getLength, null);
        });
    }

    public boolean isCalibrated() {
        return rotateMin != Double.NEGATIVE_INFINITY;
    }

    public boolean extendLimitTripped() {
        return extendLimit.get() == EXTENSION_LIMIT_PRESSED;
    }

    public boolean rotateLimitTripped() {
        return rotateLimit.get() == ROTATION_LIMIT_PRESSED;
    }

    // If we're extended beyond the critical length, we can't rotate below the critical angle
    public double getEffectiveRotateMin() {
        if (isCalibrated() && getLengthDelta() > ArmConfig.LIMIT_CRITICAL_LENGTH) {
            return rotateMin + ArmConfig.LIMIT_CRITICAL_ANGLE;
        }
        return rotateMin;
    }

    // If we're below the critical angle, we can't extend beyond the critical length
    public double getEffectiveExtendMax() {
        if (isCalibrated() && getAngleDelta() < ArmConfig.LIMIT_CRITICAL_ANGLE) {
            return extendMin + ArmConfig.LIMIT_CRITICAL_LENGTH;
        }
        return extendMax;
    }

    public double getAngle() {
        return rotateEncoder.getPosition();
    }

    public double getAngleDelta() {
        return isCalibrated()
            ? rotateEncoder.getPosition() - rotateMin
            : Double.NEGATIVE_INFINITY;
    }

    public double getLength() {
        return extendEncoder.getPosition();
    }

    public double getLengthDelta() {
        return isCalibrated()
            ? extendEncoder.getPosition() - extendMin
            : Double.NEGATIVE_INFINITY;
    }

    public void clearLimits() {
        rotateMin = Double.NEGATIVE_INFINITY;
        extendMin = Double.NEGATIVE_INFINITY;
        rotateMax = Double.POSITIVE_INFINITY;
        extendMax = Double.POSITIVE_INFINITY;
    }

    public void retractParkingBrake() {
        brakeRetracted = true;
    }

    public boolean calibrationComplete() {

        double rotateOutput = 0.0;
        if (rotateMin == Double.NEGATIVE_INFINITY) {
            if (rotateLimitTripped()) {
                rotateMax = rotateEncoder.getPosition() - ROTATE_TRAVEL_BUFFER;
                rotateMin = rotateMax - ROTATE_TRAVEL_MAX;
            } else {
                rotateOutput = ROTATOR_CALIBRATE_SPEED;
            }
        }
        rotateMotor.set(rotateOutput);

        double extendOutput = 0.0;
        if (extendMin == Double.NEGATIVE_INFINITY) {
            if (extendLimitTripped()) {
                extendMin = extendEncoder.getPosition() + EXTENDER_TRAVEL_BUFFER;
                extendMax = extendMin + EXTENDER_TRAVEL_MAX;
            } else {
                extendOutput = -EXTEND_CALIBRATE_SPEED;
            }
        }
        extendMotor.set(extendOutput);

        return rotateOutput == 0.0 && extendOutput == 0.0;
    }

    public void extendAt(double percentOutput) {
        if (percentOutput > 0 && getLength() > getEffectiveExtendMax()) {
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
        if (percentOutput < 0 && getAngle() < getEffectiveRotateMin()) {
            percentOutput = 0;
        }
        percentOutput = MathUtil.clamp(percentOutput, -ROTATOR_MAX_SPEED, ROTATOR_MAX_SPEED);
        rotateMotor.set(percentOutput);
    }

    @Override
    public void periodic() {
        extendBrake.set(brakeRetracted);
    }
}
