package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmCommands {

    public static Command clearLimits(ArmSubsystem arm) {
        return new InstantCommand(arm::clearLimits, arm);
    }

    public static Command extendBrake(ArmSubsystem arm) {
        return new InstantCommand(arm::extendParkingBrake, arm);
    }

    public static Command retractBrake(ArmSubsystem arm) {
        return new InstantCommand(arm::retractParkingBrake, arm);
    }
}
