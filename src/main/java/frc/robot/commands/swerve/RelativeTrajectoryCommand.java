package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/**
 * The WPILib {@link SwerveControllerCommand} is based on a trajectory built using
 * absolute field-relative coordinates. Using it requires that you reset the robot's
 * position prior to embarking on your journey.
 *
 * We don't want our position info (especially heading) to get reset every time
 * someone wants to use a trajectory. This command lets you create a relative
 * trajectory command.
 */
public class RelativeTrajectoryCommand {

    // PID constants
    public static final double PX_CONTROLLER = 1;
    public static final double PY_CONTROLLER = 1;

    // creates a controller for the rotation of the robot
    // (this is really irrelevant b/c we don't rotate)
    public static ProfiledPIDController makeThetaController() {
        Constraints constraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);
        ProfiledPIDController c = new ProfiledPIDController(1, 0, 0, constraints);
        c.enableContinuousInput(-Math.PI, Math.PI);
        return c;
    }

    // takes a starting point a series of translations, and returns a list of
    // all the absolute positions visited along the trajectory
    public static List<Translation2d> computeAbsoluteWaypoints(Pose2d start, Translation2d... points) {

        List<Translation2d> list = new ArrayList<>();
        list.add(start.getTranslation());

        Rotation2d rot = start.getRotation();
        Translation2d curr = start.getTranslation();

        // at each step we apply the next translation. BUT we have to rotate it
        // to take into account the robot's heading.
        for (Translation2d next : points) {
            curr = curr.plus(next.rotateBy(rot));
            list.add(curr);
        }

        return list;
    }

    // turns the current position of the supplied drive, and some relative waypoints,
    // into a command that navigates those specific absolute points. DON'T bind this
    // to a button; it won't be reusable as the robot moves around the field.
    private static Command makeAbsoluteCommand(SwerveDriveSubsystem drive, double maxSpeed, Translation2d... points) {

        TrajectoryConfig config = new TrajectoryConfig(maxSpeed, maxSpeed)
                .setKinematics(SwerveConfig.defaultKinematics);

        Pose2d start = drive.getPose();
        Rotation2d rotation = start.getRotation();
        List<Translation2d> waypoints = computeAbsoluteWaypoints(start, points);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(waypoints.get(0), rotation),
                waypoints.subList(1, waypoints.size()-1),
                new Pose2d(waypoints.get(waypoints.size()-1), rotation),
                config);

        return new SwerveControllerCommand(
                trajectory,
                drive::getPose,
                SwerveConfig.defaultKinematics,
                new PIDController(PX_CONTROLLER, 0, 0),
                new PIDController(PY_CONTROLLER, 0, 0),
                makeThetaController(),
                drive::setModuleStates,
                drive);
    }

    // creates a command that will, when it runs, move the supplied drive through
    // the supplied set of relative translations.
    public static Command makeCommand(SwerveDriveSubsystem drive, double maxSpeed, Translation2d... waypoints) {
        ProxyCommand p = new ProxyCommand(() -> makeAbsoluteCommand(drive, maxSpeed, waypoints));
        p.addRequirements(drive);
        return p;
    }
}
