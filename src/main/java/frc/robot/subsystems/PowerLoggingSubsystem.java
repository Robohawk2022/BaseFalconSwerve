package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.hal.PowerDistributionStickyFaults;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class PowerLoggingSubsystem extends SubsystemBase {

    public static final String [] NO_FAULTS = new String[0];

    public static final Map<String,TalonFX> talons = new HashMap<>();
    public static final Map<String,CANSparkMax> sparks = new HashMap<>();

    private final PowerDistribution pd;

    public PowerLoggingSubsystem() {

        DataLogManager.start();

        pd = new PowerDistribution(3, PowerDistribution.ModuleType.kRev);

        SmartDashboard.putData("PowerDistribution", builder -> {
            builder.addStringArrayProperty("CurrentFaults", this::getFaults, null);
            builder.addStringArrayProperty("CurrentStickyFaults", this::getStickyFaults, null);
            builder.addDoubleProperty("Voltage", pd::getVoltage, null);
            builder.addDoubleProperty("Temperature", this::getTemperatureF, null);
            builder.addDoubleProperty("TotalCurrentAmps", pd::getTotalCurrent, null);
        });
        SmartDashboard.putData("PowerChannels", builder -> {
            builder.addDoubleProperty("Channel0", () -> pd.getCurrent(0), null);
            builder.addDoubleProperty("Channel1", () -> pd.getCurrent(1), null);
            builder.addDoubleProperty("Channel2", () -> pd.getCurrent(2), null);
            builder.addDoubleProperty("Channel3", () -> pd.getCurrent(3), null);
            builder.addDoubleProperty("Channel4", () -> pd.getCurrent(4), null);
            builder.addDoubleProperty("Channel13", () -> pd.getCurrent(13), null);
            builder.addDoubleProperty("Channel14", () -> pd.getCurrent(14), null);
            builder.addDoubleProperty("Channel15", () -> pd.getCurrent(15), null);
            builder.addDoubleProperty("Channel16", () -> pd.getCurrent(16), null);
            builder.addDoubleProperty("Channel17", () -> pd.getCurrent(17), null);
            builder.addDoubleProperty("Channel18", () -> pd.getCurrent(18), null);
            builder.addDoubleProperty("Channel19", () -> pd.getCurrent(19), null);
        });
        SmartDashboard.putData("TalonCurrent", builder -> {
            for (String key : talons.keySet()) {
                TalonFX talon = talons.get(key);
                builder.addDoubleProperty(key, talon::getOutputCurrent, null);
            }
        });
        SmartDashboard.putData("SparkCurrent", builder -> {
            for (String key : sparks.keySet()) {
                CANSparkMax spark = sparks.get(key);
                builder.addDoubleProperty(key, spark::getOutputCurrent, null);
            }
        });
    }

    public double getTemperatureF() {
        return pd.getTemperature() * 1.8 + 32.0;
    }

    public String [] getFaults() {

        PowerDistributionFaults faults = pd.getFaults();
        if (faults == null) {
            return NO_FAULTS;
        }

        List<String> list = new ArrayList<>();
        if (faults.Channel0BreakerFault) list.add("Channel0BreakerFault");
        if (faults.Channel1BreakerFault) list.add("Channel1BreakerFault");
        if (faults.Channel2BreakerFault) list.add("Channel2BreakerFault");
        if (faults.Channel3BreakerFault) list.add("Channel3BreakerFault");
        if (faults.Channel4BreakerFault) list.add("Channel4BreakerFault");
        if (faults.Channel5BreakerFault) list.add("Channel5BreakerFault");
        if (faults.Channel6BreakerFault) list.add("Channel6BreakerFault");
        if (faults.Channel7BreakerFault) list.add("Channel7BreakerFault");
        if (faults.Channel8BreakerFault) list.add("Channel8BreakerFault");
        if (faults.Channel9BreakerFault) list.add("Channel9BreakerFault");
        if (faults.Channel10BreakerFault) list.add("Channel10BreakerFault");
        if (faults.Channel11BreakerFault) list.add("Channel11BreakerFault");
        if (faults.Channel12BreakerFault) list.add("Channel12BreakerFault");
        if (faults.Channel13BreakerFault) list.add("Channel13BreakerFault");
        if (faults.Channel14BreakerFault) list.add("Channel14BreakerFault");
        if (faults.Channel15BreakerFault) list.add("Channel15BreakerFault");
        if (faults.Channel16BreakerFault) list.add("Channel16BreakerFault");
        if (faults.Channel17BreakerFault) list.add("Channel17BreakerFault");
        if (faults.Channel18BreakerFault) list.add("Channel18BreakerFault");
        if (faults.Channel19BreakerFault) list.add("Channel19BreakerFault");
        if (faults.Channel20BreakerFault) list.add("Channel20BreakerFault");
        if (faults.Channel21BreakerFault) list.add("Channel21BreakerFault");
        if (faults.Channel22BreakerFault) list.add("Channel22BreakerFault");
        if (faults.Channel23BreakerFault) list.add("Channel23BreakerFault");
        if (faults.Brownout) list.add("Brownout");
        if (faults.CanWarning) list.add("CanWarning");
        if (faults.HardwareFault) list.add("HardwareFault");

        if (list.size() == 0) {
            return NO_FAULTS;
        }

        return list.toArray(new String[list.size()]);
    }

    public String [] getStickyFaults() {

        PowerDistributionStickyFaults faults = pd.getStickyFaults();
        if (faults == null) {
            return NO_FAULTS;
        }

        List<String> list = new ArrayList<>();
        if (faults.Channel0BreakerFault) list.add("Channel0BreakerFault");
        if (faults.Channel1BreakerFault) list.add("Channel1BreakerFault");
        if (faults.Channel2BreakerFault) list.add("Channel2BreakerFault");
        if (faults.Channel3BreakerFault) list.add("Channel3BreakerFault");
        if (faults.Channel4BreakerFault) list.add("Channel4BreakerFault");
        if (faults.Channel5BreakerFault) list.add("Channel5BreakerFault");
        if (faults.Channel6BreakerFault) list.add("Channel6BreakerFault");
        if (faults.Channel7BreakerFault) list.add("Channel7BreakerFault");
        if (faults.Channel8BreakerFault) list.add("Channel8BreakerFault");
        if (faults.Channel9BreakerFault) list.add("Channel9BreakerFault");
        if (faults.Channel10BreakerFault) list.add("Channel10BreakerFault");
        if (faults.Channel11BreakerFault) list.add("Channel11BreakerFault");
        if (faults.Channel12BreakerFault) list.add("Channel12BreakerFault");
        if (faults.Channel13BreakerFault) list.add("Channel13BreakerFault");
        if (faults.Channel14BreakerFault) list.add("Channel14BreakerFault");
        if (faults.Channel15BreakerFault) list.add("Channel15BreakerFault");
        if (faults.Channel16BreakerFault) list.add("Channel16BreakerFault");
        if (faults.Channel17BreakerFault) list.add("Channel17BreakerFault");
        if (faults.Channel18BreakerFault) list.add("Channel18BreakerFault");
        if (faults.Channel19BreakerFault) list.add("Channel19BreakerFault");
        if (faults.Channel20BreakerFault) list.add("Channel20BreakerFault");
        if (faults.Channel21BreakerFault) list.add("Channel21BreakerFault");
        if (faults.Channel22BreakerFault) list.add("Channel22BreakerFault");
        if (faults.Channel23BreakerFault) list.add("Channel23BreakerFault");
        if (faults.Brownout) list.add("Brownout");
        if (faults.CanWarning) list.add("CanWarning");
        if (faults.CanBusOff) list.add("CanBusOff");
        if (faults.HasReset) list.add("HasReset");

        if (list.size() == 0) {
            return NO_FAULTS;
        }

        return list.toArray(new String[list.size()]);
    }

    public static void addTalon(String name, TalonFX talon) {
        talons.put(name, talon);
    }

    public static void addSpark(String name, CANSparkMax spark) {
        sparks.put(name, spark);
    }
}
