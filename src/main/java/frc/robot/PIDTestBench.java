package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

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
