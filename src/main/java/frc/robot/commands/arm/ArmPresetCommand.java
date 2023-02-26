package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;


public class ArmPresetCommand extends CommandBase {

    // TODO calculate preset positions
    public static final double [] TRAVEL_POSITION = { 150.0, -150.0 };
    public static final double [] PICKUP_POSITION = { 2.0, 3.0 };
    public static final double [] HIGH_POSITION = { 10.0, 12.0 };
    public static final double [] MIDDLE_POSITION = { -12.0, -10.0 };
    public static final double [] LOW_POSITION = { 2.0, 3.0 };
    public static final double [] BALANCE_POSITION = { 2.0, 3.0 };
    public static final double [] LOAD_POSITION = { 2.0, 3.0 };

    // TODO tune me with the actual arm
    public static final double ROTATE_FAST = 0.8;
    public static final double ROTATE_SLOW = 0.2;
    public static final double ROTATE_TRANSITION_POINT = 35;
    public static final double ROTATE_TOLERANCE = 5;

    // TODO tune me with the actual arm
    public static final double EXTEND_FAST = 0.8;
    public static final double EXTEND_SLOW = 0.2;
    public static final double EXTEND_TRANSITION_POINT = 35;
    public static final double EXTEND_TOLERANCE = 5;

    public static final double MININUM_MOTOR_SPEED = 0.1;


    private final ArmSubsystem arm;
    private final double targetRotation;
    private final double targetExtension;
    private final BooleanSupplier doneSupplier;
    // private boolean done;

    public ArmPresetCommand(ArmSubsystem arm, double [] preset, BooleanSupplier doneSupplier) {
        this.arm = arm;
        this.targetRotation = preset[0];
        this.targetExtension = preset[1];
        this.doneSupplier = doneSupplier;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        // done = false;
    }

    @Override
    public void execute() {

        double rotationError = targetRotation - arm.getRotatorPosition();
        double percentRotate = calculateRotationSpeed(Math.abs(rotationError));
        if (rotationError > 0.0) {
            percentRotate = -percentRotate;
            
        }

        double extensionError = targetExtension - arm.getExtenderPosition();
        double percentExtend = calculateExtensionSpeed(Math.abs(extensionError));
        if (extensionError > 0.0) {
            percentExtend = -percentExtend;
        }

        arm.moveAt(percentRotate, percentExtend);
    }

    @Override
    public boolean isFinished() {
        return doneSupplier.getAsBoolean();
    }


    

    // TODO replace me with a ratio or a PID controller?

    private double calculateRotationSpeed(double absoluteError) {
        if (absoluteError < ROTATE_TOLERANCE) {
            return 0.0;
        }
        if (absoluteError < ROTATE_TRANSITION_POINT) {
             return ROTATE_FAST * (absoluteError - ROTATE_TOLERANCE) / (ROTATE_TRANSITION_POINT - ROTATE_TOLERANCE) + MININUM_MOTOR_SPEED;
        }
        return ROTATE_FAST;
    }

    // TODO replace me with a ratio or a PID controller?
    private double calculateExtensionSpeed(double absoluteError) {
        if (absoluteError < EXTEND_TOLERANCE) {
            return 0.0;
        }
        if (absoluteError < EXTEND_TRANSITION_POINT) {
            return EXTEND_FAST * (absoluteError - EXTEND_TOLERANCE) / (EXTEND_TRANSITION_POINT - EXTEND_TOLERANCE) + MININUM_MOTOR_SPEED;
        }
        return EXTEND_FAST;
    }
    
}
