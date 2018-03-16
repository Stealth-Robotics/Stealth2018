// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package org.usfirst.frc4089.Stealth2018.commands;

import org.usfirst.frc4089.Stealth2018.RobotMap;
import org.usfirst.frc4089.Stealth2018.MPPaths.*;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.*;

/**
 *
 */
public class Position5Path4 extends CommandGroup {
  public Position5Path4() {
    
  }

  // Called just before this Command runs the first time
  @Override
    protected void initialize() {
	  System.out.println("Position Five Source: Commands.PositionFive");
    //hug block
    addSequential(new HugBlock());
    
    //get game data
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    int counter = 0;
    while ((gameData == "" || gameData == null || gameData.length() != 3) && counter < 250) {
      gameData = DriverStation.getInstance().getGameSpecificMessage();
      counter ++;
    }
    
    boolean left = true;
    
    if(gameData.length()>1)
    {
      if('R'==gameData.charAt(1))
      {
        left = false;
      }
    }
    
    
    
    if(left)
    {
      addSequential(new DrivePathAction(new Red54Path60InPerSec()));
      System.out.println("Left");
    }
    else
    {
      addSequential(new DrivePathAction(new Red53Path60InPerSec()));
      System.out.println("Right");
    }
    //lower picker
    addParallel(new LowerPicker());
    //raise block to top
    addParallel(new RaisePickerToTop());
    
    addParallel(new RaiseMainToTop());
    //let go of block
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