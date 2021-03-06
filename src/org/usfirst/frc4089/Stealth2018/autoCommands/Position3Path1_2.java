// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package org.usfirst.frc4089.Stealth2018.autoCommands;

import org.usfirst.frc4089.Stealth2018.Robot;
import org.usfirst.frc4089.Stealth2018.RobotMap;
import org.usfirst.frc4089.Stealth2018.MPPaths.*;
import org.usfirst.frc4089.Stealth2018.commands.*;
import org.usfirst.frc4089.Stealth2018.subsystems.Picker;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.*;

/**
 *
 */
public class Position3Path1_2 extends CommandGroup {
	
	Path retracePath;
	
  public Position3Path1_2() {
    
  }

  // Called just before this Command runs the first time
  @Override
    protected void initialize() {
	  Robot.logging.LogEvent("Position3Path1_2 Source: autoCommands.Position3Path1_2");
	  //System.out.println("Position three Source: Commands.PositionThree");
	  
	  addSequential(new SetAutoFinished(false));
    //hug block
    addSequential(new HugBlock());
    //lower picker
    addSequential(new LowerPicker());
    //raise block to top
    addParallel(new  RaisePickerToTop());

    
    //get game data
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    int counter = 0;
    while ((gameData == "" || gameData == null || gameData.length() != 3) && counter < 250) {
      gameData = DriverStation.getInstance().getGameSpecificMessage();
      counter ++;
    }
    //figure out where to go
    boolean left = true;
    
    if(gameData.length()>1)
    {
      if('R'==gameData.charAt(0))
      {
        left = false;
      }
    }
    
    
    if(left)
    {
      //go to where we need to go
      addSequential(new DrivePathAction(new Red31Path60InPerSec()));
      System.out.println("Left");
    }
    else
    {
      //go to where we need to go
      addSequential(new DrivePathAction(new Red32Path60InPerSec()));
      System.out.println("Right");
    }
    
    //let go of block
    addSequential(new RejectBlock());
    
    //Robot.drive.DriveRobot(-0.3, 0);
    addSequential(new AutoRawDrive(-0.3, 0));
    
    addSequential(new WaitTime(500));
    
    addSequential(new LowerMainToBottom());
    addSequential(new LowerPickerToBottom());
    
    addSequential(new AutoFindCube());
    
    addSequential(new HugBlock());
    
    addSequential(new RaisePickerToTop());
    
    addSequential(new DrivePathAction(Robot.path));
    
    //Robot.drive.DriveRobot(0.3, 0);
    addSequential(new AutoRawDrive(0.3,0));
    
    addSequential(new WaitTime(500));
    
    addSequential(new RejectBlock());
    
    addSequential(new SetAutoFinished());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return RobotMap.isAutoFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
	  end();
  }
}
