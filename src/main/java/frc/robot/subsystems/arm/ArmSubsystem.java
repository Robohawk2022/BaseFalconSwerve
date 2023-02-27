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
    private boolean rotateNeedsCalibration;

    private final CANSparkMax extendMotor;
    private final RelativeEncoder extendEncoder;
    private final DigitalInput extendLimit;
    private final Solenoid extendBrake;
    private boolean extendNeedsCalibration;

    public ArmSubsystem() {

        rotateMotor = new CANSparkMax(ROTATION_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
        rotateMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rotateMotor.setOpenLoopRampRate(ROTATOR_RAMP_RATE);
        rotateMotor.setClosedLoopRampRate(ROTATOR_RAMP_RATE);
        rotateMotor.setInverted(ROTATION_INVERTED);
        rotateMotor.setSmartCurrentLimit(ROTATION_MAX_CURRENT);
        rotateEncoder = rotateMotor.getEncoder();
        rotateEncoder.setPositionConversionFactor(ROTATION_FACTOR);
        rotateLimit = new DigitalInput(ROTATION_LIMIT_ID);

        extendMotor = new CANSparkMax(EXTENSION_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
        extendMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        extendMotor.setOpenLoopRampRate(EXTENDER_RAMP_RATE);
        extendMotor.setClosedLoopRampRate(EXTENDER_RAMP_RATE);
        extendMotor.setInverted(EXTENSION_INVERTED);
        extendMotor.setSmartCurrentLimit(EXTENSION_MAX_CURRENT);
        extendEncoder = extendMotor.getEncoder();
        extendEncoder.setPositionConversionFactor(EXTENSION_FACTOR);
        extendLimit = new DigitalInput(EXTENSION_LIMIT_ID);

        extendBrake = new Solenoid(5, PneumaticsModuleType.REVPH, EXTENSION_BRAKE_CHANNEL);

        clearLimits();

        SmartDashboard.putData("Rotator", builder -> {
            builder.addDoubleProperty("Current", this::getAngle, null);
            builder.addBooleanProperty("NeedsCalibration", () -> rotateNeedsCalibration, null);
        });
        SmartDashboard.putData("Extender", builder -> {
            builder.addDoubleProperty("Current", this::getLength, null);
            builder.addBooleanProperty("NeedsCalibration", () -> extendNeedsCalibration, null);
        });
    }

    public double getAngle() {
        return rotateEncoder.getPosition();
    }

    public double getLength() {
        return extendEncoder.getPosition();
    }

    public void clearLimits() {
        rotateNeedsCalibration = true;
        extendNeedsCalibration = true;
    }

    public void retractParkingBrake(){
        extendBrake.set(true);
    }

    public boolean calibrate() {

        double rotateOutput = 0.0;
        if (rotateNeedsCalibration) {
            if (rotateLimit.get()) {
                rotateEncoder.setPosition(ROTATE_PHYSICAL_MAX);
                rotateNeedsCalibration = false;
            } else {
                rotateOutput = ROTATOR_MIN_SPEED;
            }
        }
        rotateAt(rotateOutput);

        double extendOutput = 0.0;
        if (extendNeedsCalibration) {
            if (extendLimit.get()) {
                extendEncoder.setPosition(EXTENDER_PHYSICAL_MIN);
                extendNeedsCalibration = false;
            } else {
                extendOutput = -EXTENDER_MAX_SPEED;
            }
        }
        extendAt(extendOutput);

        return rotateNeedsCalibration || extendNeedsCalibration;
    }

    public void extendAt(double percentOutput) {
        if (!extendNeedsCalibration) {
            if (percentOutput > 0 && getLength() > EXTENDER_TRAVEL_MAX) {
                percentOutput = 0;
            }
            if (percentOutput < 0 && getLength() < EXTENDER_TRAVEL_MIN) {
                percentOutput = 0;
            }
        }
        extendMotor.set(MathUtil.clamp(percentOutput, -EXTENDER_MAX_SPEED, EXTENDER_MAX_SPEED));
    }

    public void rotateAt(double percentOutput) {
        if (!rotateNeedsCalibration) {
            if (percentOutput > 0 && getAngle() > ROTATE_TRAVEL_MAX) {
                percentOutput = 0;
            }
            if (percentOutput < 0 && getAngle() < ROTATE_TRAVEL_MIN) {
                percentOutput = 0;
            }
        }
        rotateMotor.set(MathUtil.clamp(percentOutput, -ROTATOR_MAX_SPEED, ROTATOR_MAX_SPEED));
    }
}
