package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmCommands {

    public static Command clearLimits(ArmSubsystem arm) {
        return new InstantCommand(arm::clearLimits, arm);
    }

    public static Command retractBrake(ArmSubsystem arm) {
        return new InstantCommand(arm::retractParkingBrake, arm);
    }

    public static Command safePreset(ArmSubsystem arm, double [] preset) {
        return Commands.race(
            new WaitCommand(5.0),
            new ArmPresetCommand(arm, preset));
    }
    public static Command safeCalibrate(ArmSubsystem arm) {
        return Commands.race(
            new WaitCommand(7.0),
            new ArmCalibrationCommand(arm));
    }
}