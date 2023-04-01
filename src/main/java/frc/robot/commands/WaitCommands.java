package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;


public class WaitCommands extends CommandBase {

    private double startTime;
    private double time;
    private double finishTime;

    private boolean done;

    private Robot robot;

    public WaitCommands(Robot robot, double time){

        this.time = time;
        this.robot = robot;

    }

    public void initialized(){

        done = false;
        startTime = Timer.getFPGATimestamp();
        finishTime = time + startTime;


    }

    public void execute(){

        if (Timer.getFPGATimestamp() >= finishTime){

            done = true;

        }
    }

    public boolean isFinished(){

        return done;

    }


    
}
