package frc.robot.subsystems.arm;

import frc.robot.util.HalfBakedSpeedController;

public class ArmConfig {

    // when the arm is at its lowest level, how far can it extend
    // before it violates the rules?
    public static final double LIMIT_CRITICAL_LENGTH = 18.5;

    // when the arm is at its maximum extend, how far down can it
    // rotate before it violates the rules?
    public static final double LIMIT_CRITICAL_ANGLE = 35;
    
    // how far (in degrees) before it hits the physical stop should
    // the arm stop rotating? and how far can it rotate down from
    // that point before it hits bottom?
    public static final double ROTATE_TRAVEL_BUFFER = 5;
    public static final double ROTATE_TRAVEL_MAX = 68;

    // how far (in inches) before it hits the physical stop should
    // the arm stop retracting? and how far can it rotate down from
    // that point before it's gone too far?
    public static final double EXTENDER_TRAVEL_MAX = 25;
    public static final double EXTENDER_TRAVEL_BUFFER = 1;

    // ID assignments
    public static final int EXTENSION_CANID = 1;
    public static final int EXTENSION_LIMIT_ID = 1;
    public static final int EXTENSION_BRAKE_CHANNEL = 6;

    // properties of the physical robot
    public static final boolean EXTENSION_INVERTED = false;
    public static final double EXTENSION_FACTOR = 0.316;
    public static final int EXTENSION_MAX_CURRENT = 35;
    public static final boolean EXTENSION_LIMIT_PRESSED = false;

    // speed parameters
    public static final double EXTEND_CALIBRATE_SPEED = 0.2;
    public static final double EXTENDER_RAMP_RATE = 0.3;
    public static final double EXTENDER_MIN_SPEED = 0.1;
    public static final double EXTENDER_MAX_SPEED = 0.4;
    public static final double EXTENDER_MIN_THRESHOLD = 0.25;
    public static final double EXTENDER_MAX_THRESHOLD = 4;

    public static HalfBakedSpeedController makeExtenderSpeedController() {
        return new HalfBakedSpeedController(
                EXTENDER_MIN_THRESHOLD, EXTENDER_MAX_THRESHOLD,
                EXTENDER_MIN_SPEED, EXTENDER_MAX_SPEED);
    }

    // ID assignments
    public static final int ROTATION_CANID = 2;
    public static final int ROTATION_LIMIT_ID = 0;

    // properties of the physical robot
    public static final boolean ROTATION_INVERTED = true;
    public static final double ROTATION_FACTOR = 0.5694;
    public static final int ROTATION_MAX_CURRENT = 23;
    public static final boolean ROTATION_LIMIT_PRESSED = false;

    // speed parameters
    public static final double ROTATOR_CALIBRATE_SPEED = 0.3;
    public static final double ROTATOR_RAMP_RATE = 0.3;
    public static final double ROTATOR_MIN_SPEED = 0.075;
    public static final double ROTATOR_MAX_SPEED = 0.3;
    public static final double ROTATOR_MIN_THRESHOLD = 2;
    public static final double ROTATOR_MAX_THRESHOLD = 7;

    public static HalfBakedSpeedController makeRotatorSpeedController() {
        return new HalfBakedSpeedController(
            ROTATOR_MIN_THRESHOLD, ROTATOR_MAX_THRESHOLD,
            ROTATOR_MIN_SPEED, ROTATOR_MAX_SPEED);
    }

     public static HalfBakedSpeedController makePresetRotatorSpeedController() {
        return new HalfBakedSpeedController(
            ROTATOR_MIN_THRESHOLD, ROTATOR_MAX_THRESHOLD,
            ROTATOR_MIN_SPEED, ROTATOR_MAX_SPEED * 2);
    }


}
