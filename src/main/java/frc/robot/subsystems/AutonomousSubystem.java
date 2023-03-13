package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.HandCommands;
import frc.robot.commands.arm.ArmCommands;
import frc.robot.commands.arm.ArmPresetCommand;
import frc.robot.commands.swerve.PathPlanningCommand;

import java.util.HashMap;
import java.util.Map;

public class AutonomousSubystem extends SubsystemBase{
    
    public static final int CHANNEL_A = 6;
    public static final int CHANNEL_B = 7;
    public static final int CHANNEL_C = 8;
    public static final int CHANNEL_D = 9;

    public static final String NONE = "None";
    public static final String DROP = "Drop";
    public static final String EXIT = "Drop & Exit";
    public static final String MOUNT_ONLY = "Mount Only";
    public static final String MOUNT_C = "Exit Center & Mount";
    public static final String JUKE_L = "Juke Left & Mount";
    public static final String JUKE_R = "Juke Right & Mouint";

    // these correspond to PathPlanner files in the src/main/deploy directory,
    // without the ".path" extension
    public static final Map<String,String> PATH_NAMES;
    static {
        PATH_NAMES = new HashMap<>();
        PATH_NAMES.put(EXIT, "exit-and-turn");
        PATH_NAMES.put(MOUNT_C, "exit-mount");
        PATH_NAMES.put(JUKE_L, "juke-left");
        PATH_NAMES.put(JUKE_R, "juke-right");
    }

    public static final Object [][] programMapping = {
        { NONE, 7 },
        { DROP, 8 },
        { EXIT, 9 },
        { MOUNT_C, 10 },
        { JUKE_L, 11 },
        { JUKE_R, 12 },
        { MOUNT_ONLY, 13 },
    };
    
    private DigitalInput [] inputs;

    public AutonomousSubystem() {
        inputs = new DigitalInput[4];
        inputs[0] = new DigitalInput(CHANNEL_A);
        inputs[1] = new DigitalInput(CHANNEL_B);
        inputs[2] = new DigitalInput(CHANNEL_C);
        inputs[3] = new DigitalInput(CHANNEL_D);

        SmartDashboard.putData("AutonomusSubystem", builder -> {
            builder.addDoubleProperty("HardwareValue", this::getValue, null);
            builder.addStringProperty("ProgramName", this::getProgramName, null);
            builder.addBooleanProperty("Channel6", inputs[0]::get, null);
            builder.addBooleanProperty("Channel7", inputs[1]::get, null);
            builder.addBooleanProperty("Channel8", inputs[2]::get, null);
            builder.addBooleanProperty("Channel9", inputs[3]::get, null);

        });
    }

    public int getValue() {
        int value = 0;
        if (!inputs[0].get()) { value += 1; }
        if (!inputs[1].get()) { value += 2; }
        if (!inputs[2].get()) { value += 4; }
        if (!inputs[3].get()) { value += 8; }
        return value;
    }

    public String getProgramName() {
        int current = getValue();
        for (Object[] mapping:programMapping) {
            int value = (Integer) mapping[1];
            if (current == value) {
                return (String) mapping[0];
            }
        }
        return DROP;
    }

    public Command createCommand(Robot robot) {

        String which = getProgramName();
        SequentialCommandGroup group = new SequentialCommandGroup();

        // not doing anything? we're done!
        if (NONE.equals(which)) {
            return group;
        }

        // otherwise we're going to drop; make sure that happens
        group.addCommands(
                ArmCommands.safePreset(robot.arm, ArmPresetCommand.MIDDLE_POSITION),
                HandCommands.release(robot.hand),
                Commands.waitSeconds(1));

        // not moving anywhere? we're done!
        if (DROP.equals(which)) {
            return group;
        }

        // otherwise, look up the appropriate movement path and make it happen
        // (don't forget to raise the arm while we're moving)
        ParallelCommandGroup moves = new ParallelCommandGroup();
        moves.addCommands(new ArmPresetCommand(robot.arm, ArmPresetCommand.TRAVEL_POSITION));
        moves.addCommands(PathPlanningCommand.loadPath(
                robot.swerveDrive,
                PATH_NAMES.get(which),
                1.5));
        group.addCommands(moves);
        return group;
    }
}
