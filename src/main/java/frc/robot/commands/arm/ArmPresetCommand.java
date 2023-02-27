package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.util.HalfBakedSpeedController;

import static frc.robot.subsystems.arm.ArmConfig.*;

public class ArmPresetCommand extends CommandBase {

    // TODO calculate preset positions
    public static final double [] TRAVEL_POSITION = { 150.0, -150.0 };
    public static final double [] PICKUP_POSITION = { 2.0, 3.0 };
    public static final double [] HIGH_POSITION = { 10.0, 12.0 };
    public static final double [] MIDDLE_POSITION = { -12.0, -10.0 };
    public static final double [] LOW_POSITION = { 2.0, 3.0 };
    public static final double [] BALANCE_POSITION = { 2.0, 3.0 };
    public static final double [] LOAD_POSITION = { 2.0, 3.0 };

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

        double rotationError = targetAngle - arm.getAngle();
        double percentRotate = rotateSpeed.calculate(rotationError);
        arm.rotateAt(percentRotate);

        double extensionError = targetLength - arm.getLength();
        double percentExtend = extendSpeed.calculate(extensionError);
        arm.extendAt(percentExtend);

        done = (percentExtend == 0.0 && percentRotate == 0.0);
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
