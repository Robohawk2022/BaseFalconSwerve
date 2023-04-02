package frc.robot.commands.swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class PathPlanningCommand {

    public static Command loadPath(SwerveDriveSubsystem drive, String name, double maxSpeed) {

        double maxAccel = 0.75 * maxSpeed;
        PathConstraints constraints = new PathConstraints(maxSpeed, maxAccel);
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(name, constraints);

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                drive::getPose,
                drive::resetPose,
                SwerveConfig.defaultKinematics,
                new PIDConstants(4, 0, 0),
                new PIDConstants(4, 0, 0),
                drive::setModuleStates,
                new HashMap<>(),
                drive);
        return autoBuilder.fullAuto(pathGroup);
        // return wrapCommand(name, autoBuilder.fullAuto(pathGroup), pathGroup);
    }

    public static Command wrapCommand(String name, Command auto, List<PathPlannerTrajectory> path) {
        
        if (path.size() != 1) {
            return auto;
        }

        PathPlannerTrajectory trajectory = path.get(0);
        if (trajectory == null) {
            DriverStation.reportWarning(name+" has no trajectory", false);
            return auto;
        }

        PathPlannerState end = trajectory.getEndState();
        if (end == null) {
            DriverStation.reportWarning(name+" has no end state", false);
            return auto;
        }

        double time = end.timeSeconds;
        if (Double.isNaN(time) || time == 0) {
            DriverStation.reportWarning(name+" has invalid end time", false);
            return auto;
        }

        SmartDashboard.putNumber("Delay", time);

        //return Commands.waitSeconds(time).deadlineWith(auto);
        return auto.withTimeout(time);
    }
}
