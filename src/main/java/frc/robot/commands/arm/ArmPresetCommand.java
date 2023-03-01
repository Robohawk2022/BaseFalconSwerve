package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.util.HalfBakedSpeedController;

import static frc.robot.subsystems.arm.ArmConfig.makeRotatorSpeedController;
import static frc.robot.subsystems.arm.ArmConfig.makeExtenderSpeedController;

public class ArmPresetCommand extends CommandBase {

    // TODO calculate preset positions (degrees, inches)
    public static final double [] TRAVEL_POSITION = { 60.0, 2.0 };
    public static final double [] PICKUP_POSITION = { 5.0, 2.0 };
    public static final double [] HIGH_POSITION = { 35.0, 15.0 };
    public static final double [] MIDDLE_POSITION = { 35.0, 10.0 };
    public static final double [] BALANCE_POSITION = { 60.0, 2.0 };
    public static final double [] LOAD_POSITION = { 35.0, 10.0 };

    private final ArmSubsystem arm;
    private final double targetAngle;
    private final double targetLength;
    private final HalfBakedSpeedController rotateSpeed;
    private final HalfBakedSpeedController extendSpeed;
    private boolean done;

    public ArmPresetCommand(ArmSubsystem arm, double [] preset) {
        this.arm = arm;
        this.targetAngle = preset[0];
        this.targetLength = preset[1];
        this.rotateSpeed = makeRotatorSpeedController();
        this.extendSpeed = makeExtenderSpeedController();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
         done = false;
    }

    @Override
    public void execute() {

        // only move to a preset if the arm has been calibrated
        double currentAngle = arm.getAngleDelta();
        double currentLength = arm.getLengthDelta();
        if (currentAngle == Double.NEGATIVE_INFINITY) {
            done = true;
            return;
        }

        double rotationError = targetAngle - currentAngle;
        double percentRotate = rotateSpeed.calculate(rotationError);
        arm.rotateAt(percentRotate);

        double extensionError = targetLength - currentLength;
        double percentExtend = extendSpeed.calculate(extensionError);
        arm.extendAt(percentExtend);

        done = (percentExtend == 0.0 && percentRotate == 0.0);
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
