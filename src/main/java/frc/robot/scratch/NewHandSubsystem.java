package frc.robot.scratch;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Update to the HandSubsystem that takes into account the need to set solenoid
 * values at every interval, so they don't time out and turn off.
 */
public class NewHandSubsystem extends SubsystemBase {

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

    private final DoubleSolenoid pressure;
    private final DoubleSolenoid position;
    private final DoubleSolenoid duck;
    private boolean handOpen;
    private boolean duckExtended;

    public NewHandSubsystem(boolean startInOpenPosition) {

        pressure = new DoubleSolenoid(REVPH_CAN_ID, PneumaticsModuleType.REVPH, PRESSURE_FORWARD, PRESSURE_REVERSE);
        position = new DoubleSolenoid(REVPH_CAN_ID, PneumaticsModuleType.REVPH, POSITION_FORWARD, POSITION_REVERSE);
        duck = new DoubleSolenoid(REVPH_CAN_ID, PneumaticsModuleType.REVPH, DUCK_QUACK, DUCK_UNQUACK);
        handOpen = startInOpenPosition;
        duckExtended = startInOpenPosition;

        SmartDashboard.putData("Hand", builder -> {
            builder.addBooleanProperty("Open", () -> handOpen, null);
            builder.addBooleanProperty("Quacked", () -> duckExtended, null);
        });
    }

    public Command grab() {
        return new InstantCommand(() -> handOpen = false, this)
                .andThen(Commands.waitSeconds(0.5))
                .andThen(new InstantCommand(() -> duckExtended = false));
    }

    public Command release() {
        return new InstantCommand(() -> handOpen = true, this)
                .andThen(Commands.waitSeconds(0.5))
                .andThen(new InstantCommand(() -> duckExtended = true));
    }

    @Override
    public void periodic() {
        duck.set(duckExtended ? QUACK : UNQUACK);
        position.set(handOpen ? OPEN : CLOSED);
        pressure.set(HI);
    }
}
