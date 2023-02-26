package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmUnit;
import frc.robot.util.HalfBakedSpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

public class ArmTeleopCommand extends CommandBase {

    private final ArmSubsystem arm;
    private final ArmUnit extender;
    private final ArmUnit rotator;
    private final DoubleSupplier rotateSupplier;
    private final DoubleSupplier extendSupplier;

    private double [] wantedPosition;

    private final double EXTENDER_MAX_THRESHOLD = 2;
    private final double EXTENDER_MIN_THRESHOLD = 0.5;
    private final double ROTATOR_MAX_THRESHOLD = 2;
    private final double ROTATOR_MIN_THRESHOLD = 0.5;

    private final double MOTOR_MAX_SPEED = 0.5;
    private final double MOTOR_MIN_SPEED = 0.1;

    HalfBakedSpeedController ExtenderSpeed = new HalfBakedSpeedController(EXTENDER_MIN_THRESHOLD, EXTENDER_MAX_THRESHOLD, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
    HalfBakedSpeedController RotatorSpeed = new HalfBakedSpeedController(ROTATOR_MIN_THRESHOLD, ROTATOR_MAX_THRESHOLD, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);





    public ArmTeleopCommand(ArmSubsystem arm, DoubleSupplier rotateSupplier, DoubleSupplier extendSupplier) {
        this.arm = arm;
        this.extender = arm.extender;
        this.rotator = arm.rotator;
        this.rotateSupplier = rotateSupplier;
        this.extendSupplier = extendSupplier;
        this.wantedPosition = new double [2];

        addRequirements(arm);
        
    }

    @Override
    public void initialize(){
        
        wantedPosition[0] = arm.getExtenderPosition();
        wantedPosition[1] = arm.getRotatorPosition();
       
    }

    public void execute() {

     

        
        if (rotateSupplier.getAsDouble() > 0.1 ||
            rotateSupplier.getAsDouble() < -0.1) {
        rotator.setSafe(rotateSupplier.getAsDouble(), 1);
        wantedPosition[1] = arm.getRotatorPosition();
        }   else if (arm.getRotatorPosition() > wantedPosition[1]){
            rotator.motor.set(RotatorSpeed.calculate(wantedPosition[1] - arm.getRotatorPosition())
            );
        }   else if (arm.getRotatorPosition() < wantedPosition[1]){
            rotator.motor.set(RotatorSpeed.calculate(wantedPosition[1] - arm.getRotatorPosition())
            );
        } else {
            rotator.motor.set(0);
        }
        
        if (extendSupplier.getAsDouble() > 0.1 ||
            extendSupplier.getAsDouble() < -0.1) {
        extender.setSafe(extendSupplier.getAsDouble(), 1);
        wantedPosition[0] = arm.getExtenderPosition();
        } else if (arm.getExtenderPosition() > wantedPosition[0]){
            extender.motor.set(ExtenderSpeed.calculate(wantedPosition[0] - arm.getExtenderPosition())
            );
        } else if (arm.getExtenderPosition() < wantedPosition[0]){
            extender.motor.set(ExtenderSpeed.calculate(wantedPosition[0] - arm.getExtenderPosition())
            );
        } else {
            extender.motor.set(0);
        }
   
        SmartDashboard.putNumber("ExtenderWantedPosition", wantedPosition[0]);
        SmartDashboard.putNumber("RotatorWantedPosition", wantedPosition[1]);
        SmartDashboard.putNumber("ExtenderInput", (extendSupplier.getAsDouble()));
        SmartDashboard.putNumber("RotatorInput", (rotateSupplier.getAsDouble()));
        SmartDashboard.putNumber("ExtenderSpeed", ExtenderSpeed.calculate(wantedPosition[0] - arm.getExtenderPosition()));
        SmartDashboard.putNumber("RotatorSpeed", RotatorSpeed.calculate(wantedPosition[1] - arm.getRotatorPosition()));
      

    }
}
