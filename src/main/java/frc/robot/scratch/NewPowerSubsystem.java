package frc.robot.scratch;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.hal.PowerDistributionStickyFaults;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

public class NewPowerSubsystem extends SubsystemBase {

    public static final String [] NO_FAULTS = new String[0];

    private final PowerDistribution pd;

    public NewPowerSubsystem() {

        // TODO this may not be the correct CAN ID for the power hub
        pd = new PowerDistribution(7, PowerDistribution.ModuleType.kRev);

        SmartDashboard.putData("PowerDistribution", builder -> {
            builder.addStringArrayProperty("CurrentFaults", this::getFaults, null);
            builder.addStringArrayProperty("CurrentStickyFaults", this::getStickyFaults, null);
            builder.addDoubleProperty("InputVoltage", pd::getVoltage, null);
            builder.addDoubleProperty("Temperature", this::getTemperatureF, null);
            builder.addDoubleProperty("TotalPowerWatts", pd::getTotalPower, null);
            builder.addDoubleProperty("TotalCurrentAmps", pd::getTotalCurrent, null);
        });
    }

    public void clearStickyFaults() {
        pd.clearStickyFaults();
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
}
