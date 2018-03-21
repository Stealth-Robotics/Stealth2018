package org.usfirst.frc4089.Stealth2018.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

import java.io.FileWriter;
import java.io.IOException;

import org.usfirst.frc4089.Stealth2018.MPPaths.*;
import org.usfirst.frc4089.Stealth2018.*;

public class DrivePathAction extends Command {
    private Path mPath;
    private int mCount = 0;
    private FileWriter mLogFile;
  
    public DrivePathAction(Path path) {
      mPath = path;
      mCount = 0;
    }

    @Override
    public boolean isFinished() {
      int curCount = mPath.kNumPoints - mCount;
        if(curCount<1)
        {
          System.out.println(mPath.kNumPoints);
        }
        
        return curCount<1;
    }

    @Override
    public void execute() {
      if(mCount<mPath.kNumPoints)
      {
        Robot.drive.AutoDrive(
            mPath.kSpeed,
            mPath.kPoints[mCount][0],
            mPath.kPoints[mCount][1],
            mPath.kPoints[mCount][2],
            mLogFile);
      }
      mCount++;
    }

    @Override
    public void end() {
        Robot.drive.DriveRobot(0, 0);
    }

    @Override
    public void initialize() {
      try {
        mLogFile = new FileWriter("/home/lvuser/drivePath.csv", true);
        mLogFile.write("\n");
        mLogFile.write("\n");
        mLogFile.write("Event:"+DriverStation.getInstance().getEventName()+"\n");
        mLogFile.write("Match Number:"+DriverStation.getInstance().getMatchNumber()+"\n");
        mLogFile.write("Replay Number:"+DriverStation.getInstance().getReplayNumber()+"\n");
        mLogFile.write("Game Data:"+DriverStation.getInstance().getGameSpecificMessage()+"\n");        
        mLogFile.write("\n");
      }
      catch(IOException e) {
        e.printStackTrace();
        System.out.println("Unable to create FileWriter");
      }
      mCount = 0;
    }
}
