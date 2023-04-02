package frc.robot.commands.swerve;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class FollowPathCommand extends CommandBase {

    private final SwerveDriveSubsystem drive;
    private final PathPlannerTrajectory trajectory;
    private final PPHolonomicDriveController controller;
    private final Timer timer;
    private ChassisSpeeds targetSpeeds;
    private boolean done;

    public FollowPathCommand(SwerveDriveSubsystem drive, String pathName, double maxSpeed) {
        
        this.drive = drive;
        this.trajectory = loadTrajectory(pathName, maxSpeed);
        this.timer = new Timer();
        this.targetSpeeds = new ChassisSpeeds(0, 0, 0);
        this.controller = new PPHolonomicDriveController(
            new PIDController(1, 0, 0, 0.02),
            new PIDController(1, 0, 0, 0.02),
            new PIDController(1, 0, 0, 0.02));

        SmartDashboard.putData("SwervePathCommand", builder -> {
            builder.addDoubleProperty("DesiredX", () -> Units.metersToFeet(targetSpeeds.vxMetersPerSecond), null);
            builder.addDoubleProperty("DesiredY", () -> Units.metersToFeet(targetSpeeds.vyMetersPerSecond), null);
            builder.addDoubleProperty("DesiredOmega", () -> Units.radiansToDegrees(targetSpeeds.omegaRadiansPerSecond), null);
            builder.addBooleanProperty("Done", () -> done, null);
            builder.addStringProperty("PathName", () -> pathName, null);
            builder.addDoubleProperty("TimeCurrent", timer::get, null);
            builder.addDoubleProperty("TimeTarget", trajectory::getTotalTimeSeconds, null);
        });

        addRequirements(drive);
    }

    private PathPlannerTrajectory loadTrajectory(String pathName, double maxSpeed) {

        double maxAccel = 0.75 * maxSpeed;
        PathConstraints constraints = new PathConstraints(maxSpeed, maxAccel);
        
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, constraints);
        if (pathGroup.size() != 1) {
            throw new UnsupportedOperationException("wrong number of trajectories: "+pathGroup.size());
        }

        return pathGroup.get(0);
    }

    @Override
    public void initialize() {

        done = false;

        PathPlannerTrajectory.PathPlannerState initialState = trajectory.getInitialState();
        drive.resetPose(new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation));

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {

        double currentTime = this.timer.get();
        PathPlannerState desiredState = (PathPlannerState) trajectory.sample(currentTime);

        // ChassisSpeeds computedSpeeds = this.controller.calculate(currentPose, desiredState);
        // targetSpeeds = new ChassisSpeeds(-computedSpeeds.vxMetersPerSecond, -computedSpeeds.vyMetersPerSecond, computedSpeeds.omegaRadiansPerSecond);
        targetSpeeds = controller.calculate(drive.getPose(), desiredState);
        drive.drive(targetSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        done = this.timer.hasElapsed(trajectory.getTotalTimeSeconds());  
        return done;
    }
}
