package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class AprilTag {

    public final double id;
    public final Transform3d pose;
    public final double distance;

    public AprilTag(double id, Transform3d pose, double distance) {
        this.id = id;
        this.pose = pose;
        this.distance = distance;
    }

    // the vision system uses "z" as the forward/reverse distance to the tag.
    // the robot will drive in its x dimension to close this distance. this
    // will hopefully limit confusion when writing the commands.
    public double getForwardReverseDistance() {
//        return pose.getX();
        return distance;
    }

    // the vision system uses "X" as the left/right position of the tag.
    // the robot will drive in its Y dimension to close this distance. this
    // will hopefully limit confusion when writing the commands.
    public double getLeftRightDistance() {
        return pose.getY();
    }

    public String toString() {
        return String.format("AprilTag(id=%s,  pose=%s, distance=%s)", id, pose, distance);
    }
}
