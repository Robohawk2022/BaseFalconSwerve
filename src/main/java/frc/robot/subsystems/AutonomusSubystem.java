package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutonomusSubystem extends SubsystemBase{
    
    public static final int CHANNEL_A = 6;
    public static final int CHANNEL_B = 7;
    public static final int CHANNEL_C = 8;
    public static final int CHANNEL_D = 9;

    public static final String NONE = "None";
    public static final String DROP = "Drop";
    public static final String DROP_CENTER_EXIT = "Drop & Center Exit";
    public static final String EXIT = "Drop & Exit";
    public static final String MOUNT_L = "Mount (Left)";
    public static final String MOUNT_R = "Mount (Right)";

    public static final Object [][] programMapping = {
        {NONE, 0},
        {DROP, 1},
        {DROP_CENTER_EXIT, 2},
        {EXIT, 3},
        {MOUNT_L, 4},
        {MOUNT_R, 5},

    };

    
    private DigitalInput [] inputs;

    public AutonomusSubystem() {
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
}
