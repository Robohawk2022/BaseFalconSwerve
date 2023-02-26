package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.util.HalfBakedSpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.arm.ArmConfig.*;

public class ArmTeleopCommand extends CommandBase {

    private final ArmSubsystem arm;
    private final DoubleSupplier rotateSupplier;
    private final DoubleSupplier extendSupplier;
    private final HalfBakedSpeedController extenderSpeed;
    private final HalfBakedSpeedController rotatorSpeed;
    private final double [] wantedPosition;

    public ArmTeleopCommand(ArmSubsystem arm, DoubleSupplier rotateSupplier, DoubleSupplier extendSupplier) {
        this.arm = arm;
        this.rotateSupplier = rotateSupplier;
        this.extendSupplier = extendSupplier;
        this.extenderSpeed = makeExtenderSpeedController();
        this.rotatorSpeed = makeRotatorSpeedController();
        this.wantedPosition = new double [2];

        addRequirements(arm);
        
    }

    @Override
    public void initialize(){
        wantedPosition[0] = arm.getExtenderPosition();
        wantedPosition[1] = arm.getRotatorPosition();
    }

    public void execute() {

        double rotateInput = rotateSupplier.getAsDouble();
        double rotateError;
        double rotateOutput;
        if (rotateInput != 0.0) {
            rotateError = 0.0;
            rotateOutput = rotateInput * ROTATOR_MAX_SPEED;
            wantedPosition[1] = arm.getRotatorPosition();
        } else {
            rotateError = wantedPosition[1] - arm.getRotatorPosition();
            rotateOutput = rotatorSpeed.calculate(rotateError);
        }
        arm.rotateAt(rotateOutput);

        SmartDashboard.putNumber("RotateTarget", wantedPosition[0]);
        SmartDashboard.putNumber("RotateInput", rotateInput);
        SmartDashboard.putNumber("RotateError", rotateError);
        SmartDashboard.putNumber("RotateOutput", rotateOutput);

        double extendInput = extendSupplier.getAsDouble();
        double extendError;
        double extendOutput;
        if (extendInput != 0.0) {
            extendError = 0.0;
            extendOutput = extendSupplier.getAsDouble() * EXTENDER_MAX_SPEED;
            wantedPosition[0] = arm.getExtenderPosition();
        } else {
            extendError = wantedPosition[0] - arm.getExtenderPosition();
            extendOutput = extenderSpeed.calculate(extendError);
        }
        arm.extendAt(extendOutput);

        SmartDashboard.putNumber("ExtendTarget", wantedPosition[0]);
        SmartDashboard.putNumber("ExtendInput", extendInput);
        SmartDashboard.putNumber("ExtendError", extendError);
        SmartDashboard.putNumber("ExtendOutput", extendOutput);
    }
}
