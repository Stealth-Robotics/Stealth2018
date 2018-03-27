package org.usfirst.frc4089.Stealth2018.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

import java.io.FileWriter;
import java.io.IOException;

import org.usfirst.frc4089.Stealth2018.MPPaths.*;
import org.usfirst.frc4089.Stealth2018.utilities.StopWatch;
import org.usfirst.frc4089.Stealth2018.*;

public class AutoRawDrive extends Command {
    private double Speed;
    private double Turn;
    private int TimeOut;
    //private boolean isFinished;
    
    //private StopWatch mWaitTime = new StopWatch(500);
  
    public AutoRawDrive(double speed, double turn) {
    	//TimeOut =  timeOut;
    	Speed = speed;
    	Turn = turn;
    }

    @Override
    public boolean isFinished() {
      //return isFinished;
    	return true;
    }

    @Override
    public void execute() {
    	Robot.drive.RawDriveRobot(Speed, Turn);
    }

    @Override
    public void end() {
        //Robot.drive.DriveRobot(0, 0);
    }

    @Override
    public void initialize() {
    	Robot.logging.LogEvent("AutoRawDrive(" + Speed + "," + Turn + ") Source: Commands.AutoRawDrive");
    	//isFinished = false;
    	//mWaitTime = new StopWatch(TimeOut);
    }
    
    @Override
    protected void interrupted() {
        end();
      }
}
