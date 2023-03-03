package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class SwerveTeleopCommand extends CommandBase {

    public static final double TURBO_FACTOR = 1.8;
    public static final double SNIPER_FACTOR = 0.7;

    public static final int YES = 0;
    public static final int NO = 1;
    public static final int DONT_CARE = 2;

    protected final SwerveDriveSubsystem swerveDrive;
    protected final DoubleSupplier pxSupplier;
    protected final DoubleSupplier pySupplier;
    private final DoubleSupplier pomegaSupplier;
    private final BooleanSupplier turboSupplier;
    private final BooleanSupplier sniperSupplier;
    private int flipSign;

    public SwerveTeleopCommand(SwerveDriveSubsystem swerveDrive,
                               DoubleSupplier pxSupplier,
                               DoubleSupplier pySupplier,
                               DoubleSupplier pomegaSupplier,
                               BooleanSupplier turboSupplier,
                               BooleanSupplier sniperSupplier) {

        this.swerveDrive = swerveDrive;
        this.pxSupplier = pxSupplier;
        this.pySupplier = pySupplier;
        this.pomegaSupplier = pomegaSupplier;
        this.turboSupplier = turboSupplier;
        this.sniperSupplier = sniperSupplier;

        this.flipSign = DONT_CARE;

        addRequirements(swerveDrive);

        SmartDashboard.putData("SwerveTeleop", builder -> {
            builder.addBooleanProperty("Turbo", turboSupplier, null);
            builder.addBooleanProperty("Sniper", sniperSupplier, null);
            builder.addDoubleProperty("px", pxSupplier, null);
            builder.addDoubleProperty("py", pySupplier, null);
            builder.addDoubleProperty("pomega", pomegaSupplier, null);
        });
    }

    @Override
    public void execute() {

        double px = pxSupplier.getAsDouble();
        double py = pySupplier.getAsDouble();
        double pomega = calculateRotation();

        if (sniperSupplier.getAsBoolean()) {
            px *= SNIPER_FACTOR;
            py *= SNIPER_FACTOR;
            pomega *= SNIPER_FACTOR;
        } else if (turboSupplier.getAsBoolean()) {
            px *= TURBO_FACTOR;
            py *= TURBO_FACTOR;
            pomega *= TURBO_FACTOR;
        }

        swerveDrive.drive(px, py, pomega);
    }

    /**
     * Calculates the correct value to use for the rotation speed. This is negated if the
     * robot is facing the player, because it's more natural in "field relative" movement.
     */
    protected double calculateRotation() {

        double pomega = pomegaSupplier.getAsDouble();

        // if the rotation has dropped to zero, I no longer care about sign-flipping
        if (pomega == 0.0) {
            flipSign = DONT_CARE;
        }

        // if I don't care about sign-flipping, I'll do whatever the heading tells me
        if (flipSign == DONT_CARE) {
            Rotation2d heading = swerveDrive.getYaw();
            if (Math.abs(heading.getDegrees()) < 90) {
                flipSign = YES;
            } else {
                flipSign = NO;
            }
        }

        // if I have an opinion, I'll honor it here
        if (flipSign == YES) {
            pomega = -pomega;
        }

        return pomega;
    }
}