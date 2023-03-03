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
    private double rotateInput;
    private double rotateOutput;
    private double rotateError;
    private double extendInput;
    private double extendOutput;
    private double extendError;

    public ArmTeleopCommand(ArmSubsystem arm, DoubleSupplier rotateSupplier, DoubleSupplier extendSupplier) {

        this.arm = arm;
        this.rotateSupplier = rotateSupplier;
        this.extendSupplier = extendSupplier;
        this.extenderSpeed = makeExtenderSpeedController();
        this.rotatorSpeed = makeRotatorSpeedController();
        this.wantedPosition = new double [2];

        addRequirements(arm);

        SmartDashboard.putData("ArmTeleop", builder -> {
            builder.addDoubleProperty("RotateTarget", () -> wantedPosition[1], null);
            builder.addDoubleProperty("RotateInput", () -> rotateInput, null);
            builder.addDoubleProperty("RotateError", () -> rotateError, null);
            builder.addDoubleProperty("RotateOutput", () -> rotateOutput, null);
            builder.addDoubleProperty("ExtendTarget", () -> wantedPosition[0], null);
            builder.addDoubleProperty("ExtendInput", () -> extendInput, null);
            builder.addDoubleProperty("ExtendError", () -> extendError, null);
            builder.addDoubleProperty("ExtendOutput", () -> extendOutput, null);
        });
    }

    @Override
    public void initialize(){
       wantedPosition[0] = arm.getLength();
       wantedPosition[1] = arm.getAngle();
    }

    public void execute() {

        rotateInput = rotateSupplier.getAsDouble();
        rotateError = 0;
        rotateOutput = 0;
        if (rotateInput == 0.0) {
            rotateError = wantedPosition[1] - arm.getAngle();
            rotateOutput = rotatorSpeed.calculate(rotateError);
        } else {
            rotateError = 0.0;
            rotateOutput = rotateInput * ROTATOR_MAX_SPEED;
            wantedPosition[1] = arm.getAngle();
        }
         arm.rotateAt(rotateOutput);

        extendInput = extendSupplier.getAsDouble();
        extendError = 0;
        extendOutput = 0;
        if (extendInput == 0.0) {
            extendError = wantedPosition[0] - arm.getLength();
            extendOutput = extenderSpeed.calculate(extendError);
        } else {
            extendError = 0.0;
            extendOutput = extendSupplier.getAsDouble() * EXTENDER_MAX_SPEED;
            wantedPosition[0] = arm.getLength();
        }
        arm.extendAt(extendOutput);

    }
}
