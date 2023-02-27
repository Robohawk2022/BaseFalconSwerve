package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

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
        done = arm.calibrationComplete();
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
