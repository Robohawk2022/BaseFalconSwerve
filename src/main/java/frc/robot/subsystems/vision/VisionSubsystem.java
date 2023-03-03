package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    public static final String TABLE_NAME = "Closest Tag";

    private final DoubleSubscriber tagId;
    private final DoubleSubscriber transX;
    private final DoubleSubscriber transY;
    private final DoubleSubscriber transZ;
    private final DoubleSubscriber distance;

    public VisionSubsystem(boolean report) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(TABLE_NAME);
        this.tagId = table.getDoubleTopic("_tag_id").subscribe(Double.NaN);
        this.transX = table.getDoubleTopic("target_pose_tx").subscribe(Double.NaN);
        this.transY = table.getDoubleTopic("target_pose_ty").subscribe(Double.NaN);
        this.transZ = table.getDoubleTopic("target_pose_tz").subscribe(Double.NaN);
        this.distance = table.getDoubleTopic("distance_value").subscribe(Double.NaN);
    }

    private boolean allNumbers(double... numbers) {
        for (int i=0; i<numbers.length; i++) {
            if (Double.isNaN(numbers[i])) {
                return false;
            }
        }
        return true;
    }

    public AprilTag getAprilTag() {

        double id = tagId.get();
        double tx = transX.get();
        double ty = transY.get();
        double tz = transZ.get();
        double d = distance.get();

        if (!allNumbers(id, tx, ty, tz, d)) {
            return null;
        }

        Translation3d t = new Translation3d(tx, ty, tz);
        return new AprilTag(id, new Transform3d(t, null), d);
    }
}
