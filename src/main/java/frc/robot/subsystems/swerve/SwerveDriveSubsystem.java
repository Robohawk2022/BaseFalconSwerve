package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase {

    private final SwerveModule [] swerveModules;
    private final AHRS navx;
    private final Field2d field;
    private SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private ChassisSpeeds lastSpeed;
    private double pitchOffset;

    public SwerveDriveSubsystem() {

        field = new Field2d();
        SmartDashboard.putData("field", field);

        swerveModules = new SwerveModule[] {
            new SwerveModule(0, SwerveConfig.frontLeft),
            new SwerveModule(1, SwerveConfig.frontRight),
            new SwerveModule(2, SwerveConfig.backLeft),
            new SwerveModule(3, SwerveConfig.backRight)
        };

        navx = new AHRS(Port.kMXP);
        zeroGyro();

        /* By pausing init for a second before setting module offsets, we avoid a bug with
         * inverting motors. See https://github.com/Team364/BaseFalconSwerve/issues/8 for
         * more info.
         */
        Timer.delay(1.0);
        for (SwerveModule module : swerveModules){
            module.resetToAbsolute();
        }

        // odometry is always based on default kinematics
        odometry = new SwerveDriveOdometry(
                SwerveConfig.defaultKinematics,
                getYaw(),
                getModulePositions());
        
        kinematics = SwerveConfig.defaultKinematics;
        lastSpeed = new ChassisSpeeds(0, 0, 0);

        SmartDashboard.putData("SwerveDrive", this);

        SmartDashboard.putData("SwerveGyro", builder -> {
            builder.addBooleanProperty("Calibrated", navx::isMagnetometerCalibrated, null);
            builder.addDoubleProperty("Pitch", this::getPitch, null);
            builder.addDoubleProperty("Yaw", () -> getYaw().getDegrees(), null);
            builder.addDoubleProperty("RawX", navx::getRawGyroX, null);
            builder.addDoubleProperty("RawY", navx::getRawGyroY, null);
            builder.addDoubleProperty("RawZ", navx::getRawGyroZ, null);
            builder.addDoubleProperty("X", () -> Units.metersToFeet(navx.getDisplacementX()), null);
            builder.addDoubleProperty("Y", () -> Units.metersToFeet(navx.getDisplacementY()), null);
        });
        SmartDashboard.putData("SwerveSpeed", builder -> {
            builder.addDoubleProperty("vx", () -> lastSpeed.vxMetersPerSecond, null);
            builder.addDoubleProperty("vy", () -> lastSpeed.vyMetersPerSecond, null);
            builder.addDoubleProperty("vomega", () -> lastSpeed.omegaRadiansPerSecond, null);
        });
    }

    public void setSwerveDrivePid (double p, double i, double d){
        for(SwerveModule module : swerveModules){
            module.setSwerveDrivePid(p, i, d);
        }
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModulePosition [] getModulePositions() {
        SwerveModulePosition [] positions = new SwerveModulePosition[4];
        for (int i=0; i<4; i++) {
            positions[i] = swerveModules[i].getPosition();
        }
        return positions;
    }

    // cheap trick: when we go into "orbit mode", we'll temporarily replace
    // the kinematics used to calculate wheel states
    public void setOrbitMode(boolean orbit) {
        kinematics = orbit
            ? SwerveConfig.orbitKinematics
            : SwerveConfig.defaultKinematics;
    }

    /**
     * Stops the drive entirely
     */
    public void stop() {
        drive(0, 0, 0);
    }

    /**
     * Runs the drive at a percentage of max speed (note that percentages
     * can be higher than 100 for "turbo" mode)
     */
    public void drive(double percentX, double percentY, double percentOmega) {
        double vx = percentX * SwerveConfig.maxLinearSpeed;
        double vy = percentY * SwerveConfig.maxLinearSpeed;
        double vomega = percentOmega * SwerveConfig.maxAngularSpeed;
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vomega, getYaw());
        drive(speeds);
    }

    /**
     * Runs the drive at a set of specified speeds
     */
    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState [] moduleStates = kinematics.toSwerveModuleStates(speeds);
        setModuleStates(moduleStates);
        lastSpeed = speeds;
    }

    /**
     * Directly sets module state for each wheel
     */
    public void setModuleStates(SwerveModuleState [] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConfig.maxWheelSpeed);
        for (SwerveModule module : swerveModules) {
            module.setDesiredState(desiredStates[module.moduleNumber], false);
        }
    }    

    public void zeroGyro() {
        navx.zeroYaw();
        navx.calibrate();
        pitchOffset = navx.getPitch();
    }

    public double getPitch() {
        //return Math.toDegrees(navx.getPitch() - pitchOffset);
        return navx.getPitch() - pitchOffset;
    }

    public Rotation2d getYaw() {
        if (navx.isMagnetometerCalibrated()) {
            return Rotation2d.fromDegrees(-navx.getFusedHeading());
        }
        return Rotation2d.fromDegrees(-(navx.getYaw())+0);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    @Override
    public void periodic() {
        odometry.update(getYaw(), getModulePositions());
        field.setRobotPose(odometry.getPoseMeters());
    }
}