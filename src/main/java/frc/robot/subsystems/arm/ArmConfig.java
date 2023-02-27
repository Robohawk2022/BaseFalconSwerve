package frc.robot.subsystems.arm;

import frc.robot.util.HalfBakedSpeedController;

public class ArmConfig {

    /*
███████╗██╗  ██╗████████╗███████╗███╗   ██╗██████╗ ███████╗██████╗
██╔════╝╚██╗██╔╝╚══██╔══╝██╔════╝████╗  ██║██╔══██╗██╔════╝██╔══██╗
█████╗   ╚███╔╝    ██║   █████╗  ██╔██╗ ██║██║  ██║█████╗  ██████╔╝
██╔══╝   ██╔██╗    ██║   ██╔══╝  ██║╚██╗██║██║  ██║██╔══╝  ██╔══██╗
███████╗██╔╝ ██╗   ██║   ███████╗██║ ╚████║██████╔╝███████╗██║  ██║
╚══════╝╚═╝  ╚═╝   ╚═╝   ╚══════╝╚═╝  ╚═══╝╚═════╝ ╚══════╝╚═╝  ╚═╝
     */

    // ID assignments
    public static final int EXTENSION_CANID = 1;
    public static final int EXTENSION_LIMIT_ID = 5;
    public static final int EXTENSION_BRAKE_CHANNEL = 1;

    // properties of the physical robot
    public static final boolean EXTENSION_INVERTED = false;
    public static final double EXTENSION_TRAVEL_LIMIT = 20;
    public static final double EXTENSION_FACTOR = 0.316;
    public static final int EXTENSION_MAX_CURRENT = 23;

    // speed parameters
    public static final double EXTENDER_RAMP_RATE = 0.3;
    public static final double EXTENDER_MIN_SPEED = 0.15;
    public static final double EXTENDER_MAX_SPEED = 0.4;
    public static final double EXTENDER_MIN_THRESHOLD = 1;
    public static final double EXTENDER_MAX_THRESHOLD = 4;
    public static final double EXTENDER_PHYSICAL_MIN = 0;
    public static final double EXTENDER_TRAVEL_MIN = 2;
    public static final double EXTENDER_TRAVEL_MAX = 22;

    public static HalfBakedSpeedController makeExtenderSpeedController() {
        return new HalfBakedSpeedController(
                EXTENDER_MIN_SPEED, EXTENDER_MAX_SPEED,
                EXTENDER_MIN_THRESHOLD, EXTENDER_MAX_THRESHOLD);
    }

/*
██████╗  ██████╗ ████████╗ █████╗ ████████╗ ██████╗ ██████╗
██╔══██╗██╔═══██╗╚══██╔══╝██╔══██╗╚══██╔══╝██╔═══██╗██╔══██╗
██████╔╝██║   ██║   ██║   ███████║   ██║   ██║   ██║██████╔╝
██╔══██╗██║   ██║   ██║   ██╔══██║   ██║   ██║   ██║██╔══██╗
██║  ██║╚██████╔╝   ██║   ██║  ██║   ██║   ╚██████╔╝██║  ██║
╚═╝  ╚═╝ ╚═════╝    ╚═╝   ╚═╝  ╚═╝   ╚═╝    ╚═════╝ ╚═╝  ╚═╝
 */

    // ID assignments
    public static final int ROTATION_CANID = 2;
    public static final int ROTATION_LIMIT_ID = 4;

    // properties of the physical robot
    public static final boolean ROTATION_INVERTED = true;
    public static final double ROTATION_TRAVEL_LIMIT = 75;
    public static final double ROTATION_FACTOR = 0.5694;
    public static final int ROTATION_MAX_CURRENT = 23;
    public static final double ROTATE_PHYSICAL_MAX = 82;
    public static final double ROTATE_TRAVEL_MAX = 78;
    public static final double ROTATE_TRAVEL_MIN = 2;

    // speed parameters
    public static final double ROTATOR_RAMP_RATE = 0.3;
    public static final double ROTATOR_MIN_SPEED = 0.05;
    public static final double ROTATOR_MAX_SPEED = 0.2;
    public static final double ROTATOR_MIN_THRESHOLD = 2;
    public static final double ROTATOR_MAX_THRESHOLD = 7;

    public static HalfBakedSpeedController makeRotatorSpeedController() {
        return new HalfBakedSpeedController(
                ROTATOR_MIN_SPEED, ROTATOR_MAX_SPEED,
                ROTATOR_MIN_THRESHOLD, ROTATOR_MAX_THRESHOLD);
    }
}
