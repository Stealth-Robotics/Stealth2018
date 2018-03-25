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

import org.usfirst.frc4089.Stealth2018.RobotMap;
import org.usfirst.frc4089.Stealth2018.MPPaths.*;
import org.usfirst.frc4089.Stealth2018.commands.*;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.*;

/**
 *
 */
public class Position1Path2 extends CommandGroup {
  public Position1Path2() {
    
  }

  // Called just before this Command runs the first time
  @Override
    protected void initialize() {
	  System.out.println("Position One Source: Commands.PositionOne");
    //hug block
    addSequential(new HugBlock());
    
    
    
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    int counter = 0;
    while ((gameData == "" || gameData == null || gameData.length() != 3) && counter < 250) {
      gameData = DriverStation.getInstance().getGameSpecificMessage();
      counter ++;
    }
    boolean scaleLeft = true;
    boolean switchLeft = true;
    
    if(gameData.length()>1)
    {
      if('R'==gameData.charAt(0))
      {
    	  switchLeft = false;
      }
      if('R'==gameData.charAt(1))
      {
        scaleLeft = false;
      }
    }
    
    if(scaleLeft)
    {
    	//lower picker
        addSequential(new LowerPicker());
        //raise block to top
        addSequential(new RaisePickerToTop());
        addSequential(new RaiseMainToTop());
        
        addSequential(new DrivePathAction(new Red12Path60InPerSec()));
        
        addSequential(new AutoRotatePickerRaiseMotor(1.0));
        addSequential(new WaitTime(500));
        addSequential(new AutoRotatePickerRaiseMotor(0));
        
        addSequential(new AutoRawDrive(0,0.1));
        addSequential(new WaitTime(750));
        addSequential(new AutoRawDrive(0,0));
        System.out.println("Left Scale");
        
        
        //drop it
        addSequential(new ShootBlock());
        //addSequential(new RejectBlock());
      
    } else if (switchLeft) {
    	//lower picker
        addSequential(new LowerPicker());
        //raise block to top
        addSequential(new RaisePickerToTop());
        
    	addSequential(new DrivePathAction(new Red11Path60InPerSec()));
        System.out.println("Left Switch");
        
        //drop it
        addSequential(new RejectBlock());
        
    } else {
        addSequential(new DrivePathAction(new Move10Path60InPerSec()));
        System.out.println("Right Scale and Switch");
        //lower picker
        addSequential(new LowerPicker());
    }
    
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
