package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class SwerveCommands {

    public static Command setOrbitMode(SwerveDriveSubsystem drive, boolean orbit) {
        return new InstantCommand(() -> drive.setOrbitMode(orbit), drive);
    }

    public static Command zeroGyro(SwerveDriveSubsystem drive) {
        return new InstantCommand(() -> drive.zeroGyro(), drive);
    }

    public static Command turnWheels(SwerveDriveSubsystem drive, double degrees) {

        final SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < states.length; i++) {
            states[i] = new SwerveModuleState(0.1, Rotation2d.fromDegrees(degrees));
        }

        return new InstantCommand(() -> drive.setModuleStates(states), drive);
    }

    public static Command scootForward(SwerveDriveSubsystem drive, double inches) {
        ChassisSpeeds speeds = new ChassisSpeeds(Units.feetToMeters(inches), 0,0);
        return new SwerveFixedSpeedCommand(drive, speeds, false, 1.0);
    }

    public static Command scootBackward(SwerveDriveSubsystem drive, double inches) {
        ChassisSpeeds speeds = new ChassisSpeeds(-Units.feetToMeters(inches), 0,0);
        return new SwerveFixedSpeedCommand(drive, speeds, false, 1.0);
    }

    public static Command scootLeft(SwerveDriveSubsystem drive, double inches) {
        ChassisSpeeds speeds = new ChassisSpeeds(0, Units.inchesToMeters(inches), 0);
        return new SwerveFixedSpeedCommand(drive, speeds, true, 1.0);
    }

    public static Command scootRight(SwerveDriveSubsystem drive, double inches) {
        ChassisSpeeds speeds = new ChassisSpeeds(0, -Units.inchesToMeters(inches), 0);
        return new SwerveFixedSpeedCommand(drive, speeds, true, 1.0);
    }
}
