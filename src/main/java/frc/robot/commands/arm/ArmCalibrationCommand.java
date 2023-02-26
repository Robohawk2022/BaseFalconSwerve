package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

import static frc.robot.subsystems.arm.ArmConfig.*;

public class ArmCalibrationCommand extends CommandBase {

    private final ArmSubsystem arm;
    private boolean done;

    public ArmCalibrationCommand(ArmSubsystem arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        done = false;
        arm.clearLimits();
    }

    @Override
    public void execute() {
        done = arm.calibrate(ROTATOR_MIN_SPEED, EXTENDER_MIN_SPEED);
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
