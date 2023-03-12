package frc.robot.subsystems;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.HandCommands;

/**
 * Represents the pneumatics for the hand. There are three double solenoids that
 * control its behavior:
 *   - PRESSURE (options: low, high or off)
 *   - POSITION (options: open, closed or off)
 *   - DUCK (options: extended or quacked, retracted or unquacked, or off)
 */
public class HandSubsystem extends SubsystemBase {

    /** CAN bus ID for the pneumatics hub */
    public static final int REVPH_CAN_ID = 5;

    /** Switch IDs on the hub for the solenoids */
    public static final int PRESSURE_FORWARD = 2;
    public static final int PRESSURE_REVERSE = 3;
    public static final int POSITION_FORWARD = 0;
    public static final int POSITION_REVERSE = 1;
    public static final int DUCK_QUACK = 4;
    public static final int DUCK_UNQUACK = 5;

    /** Solenoid values for the different options */
    public static final Value LO = Value.kReverse;
    public static final Value HI = Value.kForward;
    public static final Value OPEN = Value.kForward;
    public static final Value CLOSED = Value.kReverse;
    public static final Value QUACK = Value.kForward;
    public static final Value UNQUACK = Value.kReverse;
    public static final Value OFF = Value.kOff;

    private final Compressor compressor;
    private final DoubleSolenoid pressure;
    private final DoubleSolenoid position;
    private final DoubleSolenoid duck;
    private Value pressureValue;
    private Value positionValue;
    private Value duckValue;

    public HandSubsystem() {

        compressor = new Compressor(REVPH_CAN_ID, PneumaticsModuleType.REVPH);
        pressure = new DoubleSolenoid(REVPH_CAN_ID, PneumaticsModuleType.REVPH, PRESSURE_FORWARD, PRESSURE_REVERSE);
        position = new DoubleSolenoid(REVPH_CAN_ID, PneumaticsModuleType.REVPH, POSITION_FORWARD, POSITION_REVERSE);
        duck = new DoubleSolenoid(REVPH_CAN_ID, PneumaticsModuleType.REVPH, DUCK_QUACK, DUCK_UNQUACK);
        
        pressureValue = HI;
        positionValue = CLOSED;
        duckValue = UNQUACK;

        PowerLoggingSubsystem.addItem("Compressor", compressor::getCurrent);

        SmartDashboard.putData("Hand", builder -> {
            builder.addStringProperty("Claw", this::getClawPositionString, null);
            builder.addStringProperty("Duck", this::getDuckPositionString, null);
        });
    }

    private String getDuckPositionString() {
        Value val = duck.get();
        if (val == QUACK) return "Extended";
        if (val == UNQUACK) return "Retracted";
        return "Off";
    }

    private String getClawPositionString() {
        Value val = position.get();
        if (val == OPEN) return "Open";
        if (val == CLOSED) return "Closed";
        return "Off";
    }

    public void grabCone() {
        pressureValue = HI;
        positionValue = CLOSED;
    }

    public void release() {
        pressureValue = HI;
        positionValue = OPEN;
    }

    public void quack() {
        duckValue = QUACK;
    }

    public void unQuack() {
        duckValue = UNQUACK;
    }

    @Override
    public void periodic() {
        position.set(positionValue);
        pressure.set(pressureValue);
        duck.set(duckValue);
    }
}
