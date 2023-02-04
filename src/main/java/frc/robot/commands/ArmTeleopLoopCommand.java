package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ArmSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftX;
import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftY;
import static edu.wpi.first.wpilibj.XboxController.Axis.kRightX;
import static edu.wpi.first.wpilibj.XboxController.Axis.kRightY;
import static edu.wpi.first.wpilibj.XboxController.Button.kX;
import static edu.wpi.first.wpilibj.XboxController.Button.kY;
import static edu.wpi.first.wpilibj.XboxController.Button.kStart;

public class ArmTeleopLoopCommand extends CommandBase {

    private final ArmSubsystem arm;
    private final XboxController inputController;
    private boolean done;

    public ArmTeleopLoopCommand(ArmSubsystem armSubsystem, XboxController xboxController){

        this.arm = armSubsystem;
        this.inputController = xboxController;

    }  

    public void initialize(){

        done = false;

    }

    public void execute(){

        double rotatingMotorPosition = arm.rotatingEncoder.getPosition();
        double extendingMotorPosition = arm.extendingEncoder.getPosition();

        //Alternative: convert this to a method

        if (rotatingMotorPosition < arm.MAXIMUM_POSITION_ROTATING || rotatingMotorPosition > arm.MININUM_POSITION_EXTENDING){       
            arm.rotatingMotor.set(-inputController.getRawAxis(kLeftY.value));
        } else if (rotatingMotorPosition > arm.MAXIMUM_POSITION_ROTATING && -inputController.getRawAxis(kLeftY.value) > 0){
            arm.rotatingMotor.set(0);
        } else if (rotatingMotorPosition < arm.MININUM_POSITION_ROTATING && -inputController.getRawAxis(kLeftY.value) < 0){
            arm.rotatingMotor.set(0);
        } else {
            arm.rotatingMotor.set(-inputController.getRawAxis(kLeftY.value));
        }

        if (extendingMotorPosition < arm.MAXIMUM_POSITION_EXTENDING || extendingMotorPosition > arm.MININUM_POSITION_EXTENDING){
            arm.extendingMotor.set(-inputController.getRawAxis(kRightY.value));
        } else if (extendingMotorPosition > arm.MAXIMUM_POSITION_EXTENDING && -inputController.getRawAxis(kRightY.value) > 0){
            arm.rotatingMotor.set(0);
        } else if (rotatingMotorPosition < arm.MININUM_POSITION_EXTENDING && -inputController.getRawAxis(kRightY.value) < 0){
            arm.rotatingMotor.set(0);
        } else {
            arm.extendingMotor.set(-inputController.getRawAxis(kRightY.value));
        }

    }

    public boolean isFinished(){

        return done;
        
    }
}