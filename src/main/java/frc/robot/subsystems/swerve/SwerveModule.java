package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import frc.robot.subsystems.PowerLoggingSubsystem;

import static frc.robot.subsystems.swerve.SwerveConfig.*;

public class SwerveModule {

    public static final double MIN_SPEED_FOR_TURNING = 0.03;

    public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
            driveKS,
            driveKV,
            driveKA);

    public final int moduleNumber;
    private final Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private final TalonFX angleMotor;
    private final TalonFX driveMotor;
    private final CANCoder angleEncoder;

    public SwerveModule(int moduleNumber, ModuleConfig moduleConstants) {

        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        this.angleEncoder = createAngleEncoder(moduleConstants.cancoderID);
        this.angleMotor = createAngleMotor(moduleConstants.angleMotorID);
        this.driveMotor = createDriveMotor(moduleConstants.driveMotorID);
        this.lastAngle = getState().angle;

        PowerLoggingSubsystem.addTalon("Angle"+moduleNumber, angleMotor);
        PowerLoggingSubsystem.addTalon("Drive"+moduleNumber, driveMotor);

        SmartDashboard.putData("SwerveModule" + moduleNumber, builder -> {  
            builder.addDoubleProperty("LastAngle", () -> lastAngle.getDegrees(), null);
            builder.addDoubleProperty("EncoderAngle", () -> SwerveUtils.falconToDegrees(angleMotor.getSelectedSensorPosition(), SwerveConfig.angleGearRatio), null);
        });
    }

    /**
     * Set the desired state for this module. Uses a custom optimize function, since default
     * WPILib optimize assumes continuous controller which CTRE onboard is not
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveUtils.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState);
        //Ignoring request for open loop for now.
    }

    /**
     * Set the speed of the drive motor
     */
    private void setSpeed(SwerveModuleState desiredState) {
        double velocity = SwerveUtils.MPSToFalcon(desiredState.speedMetersPerSecond, wheelCircumference, driveGearRatio);
        driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        SmartDashboard.putNumber("motor-"+moduleNumber+"-v", velocity);
    }

    /**
     * Set the position of the angle motor. Ignores very small angle changes if the
     * speed is low to keep the wheel from "jittering".
     */
    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= MIN_SPEED_FOR_TURNING)
                ? lastAngle :
                desiredState.angle;
        angleMotor.set(ControlMode.Position, SwerveUtils.degreesToFalcon(angle.getDegrees(), angleGearRatio));
        lastAngle = angle;
    }

    public void setSwerveDrivePid (double p, double i, double d){
        angleMotor.config_kP(0, p);
        angleMotor.config_kI(0,i);
        angleMotor.config_kD(0,d);
    }


    /**
     * Get the angle of the wheel, as determined by the position of the angle motor
     * and the gearing of the wheel
     */
    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(SwerveUtils.falconToDegrees(angleMotor.getSelectedSensorPosition(), angleGearRatio));
    }

    /**
     * Get the value reported from the CANCoder
     */
    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    /**
     * Resets the angle motor based on the absolute position reported by the CANCoder
     */
    public void resetToAbsolute() {
        double d1 = getCanCoder().getDegrees() - angleOffset.getDegrees();
        double d2 = d1 % 360.0;
        System.err.println(String.format("resetting with d1 %.3f, d2 %.3f", d1, d2));
        double absolutePosition = SwerveUtils.degreesToFalcon(d2, angleGearRatio);
        angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            SwerveUtils.falconToMPS(driveMotor.getSelectedSensorVelocity(), wheelCircumference, driveGearRatio),
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            SwerveUtils.falconToMeters(driveMotor.getSelectedSensorPosition(), wheelCircumference, driveGearRatio),
            getAngle()
        );
    }
}