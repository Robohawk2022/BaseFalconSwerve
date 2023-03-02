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
    private boolean alreadyAppliedSpeedTweak;

    public ArmTeleopCommand(ArmSubsystem arm, DoubleSupplier rotateSupplier, DoubleSupplier extendSupplier) {
        this.arm = arm;
        this.rotateSupplier = rotateSupplier;
        this.extendSupplier = extendSupplier;
        this.extenderSpeed = makeExtenderSpeedController();
        this.rotatorSpeed = makeRotatorSpeedController();
        this.wantedPosition = new double [2];
        this.alreadyAppliedSpeedTweak = false;

        addRequirements(arm);
        
    }

    @Override
    public void initialize(){
       wantedPosition[0] = arm.getLength();
       wantedPosition[1] = arm.getAngle();
    }

    public void execute() {

        double rotateInput = rotateSupplier.getAsDouble();
        double extendInput = extendSupplier.getAsDouble();

        // if the user is extending, but not rotating, the arm might bind up
        // and the cable unravel. to fight this, we will add in some rotation.
        if (extendInput > 0.0 && rotateInput == 0.0) {
            if (!alreadyAppliedSpeedTweak) {
                rotateInput = 0.1;
                alreadyAppliedSpeedTweak = true;
            }
        }
        if (extendInput == 0.0) {
            alreadyAppliedSpeedTweak = false;
        }

        double rotateError = 0;
        double rotateOutput = 0;
        if (rotateInput == 0.0) {
            rotateError = wantedPosition[1] - arm.getAngle();
            rotateOutput = rotatorSpeed.calculate(rotateError);
        } else {
            rotateError = 0.0;
            rotateOutput = rotateInput * ROTATOR_MAX_SPEED;
            wantedPosition[1] = arm.getAngle();
        }
         arm.rotateAt(rotateOutput);

        SmartDashboard.putNumber("RotateTarget", wantedPosition[1]);
        SmartDashboard.putNumber("RotateInput", rotateInput);
        SmartDashboard.putNumber("RotateError", rotateError);
        SmartDashboard.putNumber("RotateOutput", rotateOutput);

        double extendError = 0;
        double extendOutput = 0;
        if (extendInput == 0.0) {
            extendError = wantedPosition[0] - arm.getLength();
            extendOutput = extenderSpeed.calculate(extendError);
        } else {
            extendError = 0.0;
            extendOutput = extendSupplier.getAsDouble() * EXTENDER_MAX_SPEED;
            wantedPosition[0] = arm.getLength();
        }
        arm.extendAt(extendOutput);

        SmartDashboard.putNumber("ExtendTarget", wantedPosition[0]);
        SmartDashboard.putNumber("ExtendInput", extendInput);
        SmartDashboard.putNumber("ExtendError", extendError);
        SmartDashboard.putNumber("ExtendOutput", extendOutput);
    }
}
