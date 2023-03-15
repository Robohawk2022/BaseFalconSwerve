package frc.robot.commands.swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import java.util.List;
import java.util.Map;

public class PathPlanningCommand {

    public static Command loadPath(SwerveDriveSubsystem drive, String name, double maxSpeed, Map<String,Command> events) {

        double maxAccel = 0.75 * maxSpeed;
        PathConstraints constraints = new PathConstraints(maxSpeed, maxAccel);
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(name, constraints);

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                drive::getPose,
                drive::resetPose,
                SwerveConfig.defaultKinematics,
                new PIDConstants(5, 0, 0),
                new PIDConstants(5, 0, 0),
                drive::setModuleStates,
                events,
                drive);
        return autoBuilder.followPathGroup(pathGroup);
    }
}
