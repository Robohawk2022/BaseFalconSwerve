package frc.robot;

import javax.xml.xpath.XPathVariableResolver;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.HandCommands;
import frc.robot.commands.arm.ArmCalibrationCommand;
import frc.robot.commands.arm.ArmCommands;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import frc.robot.PIDControlMapping;

public class PIDTestBench extends TimedRobot {
    public static final int DRIVE_PORT = 0;


    private XboxController xboxController;

    private PIDController controller;

    public SwerveDriveSubsystem swerveDrive;
    public PIDControlMapping mapping;

    double inip; 
    double inii;
    double inid;
    

    public void RobotInit(){
        controller = new PIDController(0.3, 0.0, 0.0);

        inip = controller.getP();
        inii = controller.getI();
        inid = controller.getD();

        swerveDrive = new SwerveDriveSubsystem();
        xboxController = new XboxController(DRIVE_PORT);
        

        SmartDashboard.putData("pid", controller);


    }

    public void testPeriodic(){

        double p = controller.getP();
        double i = controller.getI();
        double d = controller.getD();

        if(p != inip || i != inii || d != inid){
        swerveDrive.setSwerveDrivePid(p, i, d);
        inip = p;
        inii = i;
        inid = d;
        }

        double rotate = -xboxController.getRightX() * 90;
        
        SwerveModuleState [] state = new SwerveModuleState[4];

        for(int c = 0; c < 4; c++){
            state[c] = new SwerveModuleState(0.1, Rotation2d.fromDegrees(rotate));
        }

        swerveDrive.setModuleStates(state);
 

    }
}
