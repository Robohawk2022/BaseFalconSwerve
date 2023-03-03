package frc.robot.commands.swerve;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Implements a simplification for high-speed movement. Above a certain speed, we will
 * map the joystick angle into an "octant". Each octant is a 45-degree slice of the
 * pie, with the 0th octant centered on the negative X axis.
 */
public class SwerveTeleopOctantCommand extends SwerveTeleopCommand {

    public static final double SWITCH_THRESHOLD = 0.4;
    public static final double DEG_45 = Units.degreesToRadians(45);
    public static final double DEG_200_5 = Units.degreesToRadians(200.5);

    public static final double [][] OCTANT_SPEEDS = {
            { -1.0, 0.0 },
            { -1.0, -1.0 },
            { 0.0, -1.0 },
            { 1.0, -1.0 },
            { 1.0, 0.0 },
            { 1.0, 1.0 },
            { 0.0, 1.0 },
            { -1.0, 1.0 }
    };

    public SwerveTeleopOctantCommand(SwerveDriveSubsystem swerveDrive,
                                     DoubleSupplier pxSupplier,
                                     DoubleSupplier pySupplier,
                                     DoubleSupplier pomegaSupplier,
                                     BooleanSupplier turboSupplier,
                                     BooleanSupplier sniperSupplier) {
        super(swerveDrive, pxSupplier, pySupplier, pomegaSupplier, turboSupplier, sniperSupplier);
    }

    @Override
    public void execute() {

        double px = pxSupplier.getAsDouble();
        double py = pySupplier.getAsDouble();
        double pomega = calculateRotation();

        if (px != 0.0 || py != 0.0) {

            double speed = Math.sqrt(px * px + py * py);
            if (speed > SWITCH_THRESHOLD) {

                double theta = Math.atan2(py, px);
                theta += DEG_200_5;
                int octant = (int) (Math.floor(theta / DEG_45)) % 8;

                px = OCTANT_SPEEDS[octant][0] * speed;
                py = OCTANT_SPEEDS[octant][1] * speed;
            }
        }

        swerveDrive.drive(px, py, pomega);
    }
}