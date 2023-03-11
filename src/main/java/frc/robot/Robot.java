package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.HandCommands;
import frc.robot.commands.arm.ArmCalibrationCommand;
import frc.robot.commands.arm.ArmCommands;
import frc.robot.subsystems.AutonomusSubystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

/**
 * This is where all the parts of our robot are created.
 * 
 * DON'T add new control logic to this class. Instead, do the following:
 * 
 *  - Create a new command that will wrap up your logic. See ExampleCommand.java
 *  in the commands folder, or look at some of the other commands in there to
 *  see how they work.
 * 
 *  - Update RobotControlMapping.java to establish a button or something that
 *  will trigger your command.
 */
public class Robot extends TimedRobot {

    public static final int DRIVE_PORT = 0;
    public static final int OPS_PORT = 1;

    public AutonomusSubystem auto;
    public SwerveDriveSubsystem swerveDrive;
    public HandSubsystem hand;
    public ArmSubsystem arm;
    public VisionSubsystem vision;
    public Command autonomousCommand;
    public RobotControlMapping mapping;
    public boolean initRun;

    @Override
    public void robotInit() {

        auto = new AutonomusSubystem();

        // create the swerve drive and establish the default control mapping
        // for driving in teleop mode
        swerveDrive = new SwerveDriveSubsystem();
        vision = new VisionSubsystem(true);
        hand = new HandSubsystem();
        arm = new ArmSubsystem();
        initRun = false;

        // do any additional control mapping that needs to be done
        mapping = new RobotControlMapping(
                this,
                new CommandXboxController(DRIVE_PORT),
                new CommandXboxController(OPS_PORT));

        

        CameraServer.startAutomaticCapture();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
 
        autonomousCommand = AutonomousCommand.generateProgram(this, auto.getProgramName());
        if (!initRun) {
            autonomousCommand = initCommand().andThen(autonomousCommand);
        }
        autonomousCommand.schedule();
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
            autonomousCommand = null;
        }
        if (!initRun) {
            initCommand().schedule();
        }
        swerveDrive.zeroGyro();
    }

    private Command initCommand() {

        Command armInit = Commands.sequence(
                ArmCommands.retractBrake(arm),
                new WaitCommand(0.5),
                new ArmCalibrationCommand(arm));

        return Commands.sequence(
                Commands.parallel(
                        armInit,
                        HandCommands.grab(hand)),
                new InstantCommand(() -> initRun = true));
    }
}
