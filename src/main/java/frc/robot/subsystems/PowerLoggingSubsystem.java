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
import java.util.Set;
import java.util.function.DoubleSupplier;

public class PowerLoggingSubsystem extends SubsystemBase {

    public static final String [] NO_FAULTS = new String[0];

    public static final Map<String,DoubleSupplier> items = new HashMap<>();

    private final PowerDistribution pd;
    private boolean brownout;
    private boolean canWarning;
    private boolean canOff;
    private boolean hasReset;
    private Set<Integer> breakerFaults;
    private double temperatureF;

    public PowerLoggingSubsystem() {

        DataLogManager.start();

        pd = new PowerDistribution(3, PowerDistribution.ModuleType.kRev);

        periodic();

        SmartDashboard.putData("PowerHub", builder -> {
            builder.addStringProperty("BreakerFaults", breakerFaults::toString, null);
            builder.addBooleanProperty("Brownout?", () -> brownout, null);
            builder.addBooleanProperty("CAN Off?", () -> canOff, null);
            builder.addBooleanProperty("CAN Warning?", () -> canWarning, null);
            builder.addBooleanProperty("Has Reset?", () -> hasReset, null);
            builder.addDoubleProperty("Voltage", pd::getVoltage, null);
            builder.addDoubleProperty("Temperature", () -> temperatureF, null);
            builder.addDoubleProperty("TotalCurrentAmps", pd::getTotalCurrent, null);
        });
        SmartDashboard.putData("PowerItems", builder -> {
            for (String key : items.keySet()) {
                builder.addDoubleProperty(key, items.get(key), null);
            }
        });
    }

    public void clearStickyFaults() {
        pd.clearStickyFaults();
    }

    @Override
    public void periodic() {

        temperatureF = pd.getTemperature() * 1.8 + 32.0;

        PowerDistributionStickyFaults faults = pd.getStickyFaults();
        brownout = faults.Brownout;
        canWarning = faults.CanWarning;
        canOff = faults.CanBusOff;
        hasReset = faults.HasReset;

        breakerFaults.clear();
        if (faults.Channel0BreakerFault) breakerFaults.add(0);
        if (faults.Channel1BreakerFault) breakerFaults.add(1);
        if (faults.Channel2BreakerFault) breakerFaults.add(2);
        if (faults.Channel3BreakerFault) breakerFaults.add(3);
        if (faults.Channel4BreakerFault) breakerFaults.add(4);
        if (faults.Channel13BreakerFault) breakerFaults.add(13);
        if (faults.Channel14BreakerFault) breakerFaults.add(14);
        if (faults.Channel15BreakerFault) breakerFaults.add(15);
        if (faults.Channel16BreakerFault) breakerFaults.add(16);
        if (faults.Channel17BreakerFault) breakerFaults.add(17);
        if (faults.Channel18BreakerFault) breakerFaults.add(18);
        if (faults.Channel19BreakerFault) breakerFaults.add(19);
    }

    public static void addItem(String name, DoubleSupplier supplier) {
        items.put(name, supplier);
    }

    public static void addTalon(String name, TalonFX talon) {
        addItem(name, talon::getSupplyCurrent);
    }

    public static void addSpark(String name, CANSparkMax spark) {
        addItem(name, spark::getOutputCurrent);
    }
}
