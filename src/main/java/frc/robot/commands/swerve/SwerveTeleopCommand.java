package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class SwerveTeleopCommand extends CommandBase {

    public enum FlipSign {
        YES,
        NO,
        DONT_CARE
    }

    protected final SwerveDriveSubsystem swerveDrive;
    protected final DoubleSupplier pxSupplier;
    protected final DoubleSupplier pySupplier;
    private final DoubleSupplier pomegaSupplier;
    private FlipSign flipSign;

    public SwerveTeleopCommand(SwerveDriveSubsystem swerveDrive,
                               DoubleSupplier pxSupplier,
                               DoubleSupplier pySupplier,
                               DoubleSupplier pomegaSupplier) {

        this.swerveDrive = swerveDrive;
        this.pxSupplier = pxSupplier;
        this.pySupplier = pySupplier;
        this.pomegaSupplier = pomegaSupplier;
        this.flipSign = FlipSign.DONT_CARE;

        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {

        double px = pxSupplier.getAsDouble();
        double py = pySupplier.getAsDouble();
        double pomega = calculateRotation();

        swerveDrive.drive(px, py, pomega);
    }

    /**
     * Calculates the correct value to use for the rotation speed. This is negated if the
     * robot is facing the player, because it's more natural in "field relative" movement.
     */
    protected double calculateRotation() {

        double pomega = pomegaSupplier.getAsDouble();
        if (pomega == 0.0) {
            flipSign = FlipSign.DONT_CARE;
        }

        if (flipSign == FlipSign.DONT_CARE) {
            Rotation2d heading = swerveDrive.getYaw();
            if (Math.abs(heading.getDegrees()) < 90) {
                flipSign = FlipSign.YES;
            } else {
                flipSign = FlipSign.NO;
            }
        }

        if (flipSign == FlipSign.YES) {
            pomega = -pomega;
        }

        return pomega;
    }
}