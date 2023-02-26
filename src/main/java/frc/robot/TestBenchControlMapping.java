package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.HandCommands;
import frc.robot.commands.arm.ArmPresetCommand;
import frc.robot.commands.arm.ArmTeleopCommand;

/**
 * All of the mapping of controls to commands happens here.
 */
public class TestBenchControlMapping {

    public static final double DEADBAND = 0.1;

    public static double clean(double input) {
        return MathUtil.applyDeadband(input * Math.abs(input), DEADBAND);
    }
    
    public static void mapControls(TestBench testBench, CommandXboxController controller) {


        /*
         * DEFAULT TELEOP
         *   - left Y is rotate
         *   - right Y is extend
         */
        testBench.arm.setDefaultCommand(new ArmTeleopCommand(
            testBench.arm,
                () -> clean(controller.getLeftY()),
                () -> clean(controller.getRightY())));

        // controller.y().onTrue(HandCommands.grab(testBench.hand));
        // controller.rightBumper().onTrue(HandCommands.release(testBench.hand));
        controller.a().onTrue(new ArmPresetCommand(testBench.arm, ArmPresetCommand.HIGH_POSITION, controller.a()));
        controller.b().onTrue(new ArmPresetCommand(testBench.arm, ArmPresetCommand.MIDDLE_POSITION, controller.b()));
    }
}
