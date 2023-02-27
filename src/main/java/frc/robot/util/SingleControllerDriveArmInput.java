package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Both arm and drive control use joystick input. This provides a wrapper around
 * a single controller that uses a flag to decide whether its joysticks should
 * be treated as input for the drive or the arm.
 */
public class SingleControllerDriveArmInput {

    private final CommandXboxController controller;
    private boolean arm;

    public SingleControllerDriveArmInput(CommandXboxController controller) {
        this.controller = controller;
        this.arm = false;
    }

    public void enableArmInput(boolean arm) {
        this.arm = arm;
    }

    public double getDriveForwardReverse() {
        return arm ? 0 : controller.getLeftY();
    }

    public double getDriveStrafeLeftRight() {
        return arm ? 0 : controller.getLeftX();
    }

    public double getDriveRotateLeftRight() {
        return arm ? 0 : controller.getRightX();
    }

    public double getArmRotate() {
        return arm ? controller.getLeftY() : 0;
    }

    public double getArmExtend() {
        return arm ? controller.getRightY() : 0;
    }
}
