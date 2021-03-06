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

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.*;

/**
 *
 */
public class Position1Path2_switch extends CommandGroup {
  public Position1Path2_switch() {
    
  }

  // Called just before this Command runs the first time
  @Override
    protected void initialize() {
	  	Robot.logging.LogEvent("Position1Path2_switch Source: autoCommands.Position1Path2_switch");
	    //reset gyro
	    RobotMap.pigeonIMU.setFusedHeading(0, 30);
	    
		//set auto finished false
		addSequential(new SetAutoFinished(false));
		
	    //hug block
	    addSequential(new HugBlock());
	    
	    //get field data
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
	    
	    if (switchLeft) {
	    	//lower picker
	        addSequential(new LowerPicker());
	        //raise block to top
	        addSequential(new RaisePickerToTop());
	        //drive to the switch
	    	addSequential(new DrivePathAction(new Red11Path60InPerSec()));
	        //drop it
	        addSequential(new RejectBlock());
	    } else if(scaleLeft) {
	    	//lower picker
	        addSequential(new LowerPicker());
	        //raise block to top top
	        addSequential(new RaisePickerToTop());
	        addSequential(new RaiseMainToTop());
	        //get to the scale
	        addSequential(new DrivePathAction(new Red12Path60InPerSec()));
	        //Raise the picker a little bit
	        addSequential(new AutoRotatePickerRaiseMotor(1.0));
	        addSequential(new WaitTime(600));
	        addSequential(new AutoRotatePickerRaiseMotor(0));
	        //rotate to face the scale
	        addSequential(new AutoRawDrive(0,0.1));
	        addSequential(new WaitTime(750));
	        addSequential(new AutoRawDrive(0,0));
	        //shoot it
	        addSequential(new ShootBlock());
	      
	    } else {
	    	System.out.println("Right Scale and Switch");
	    	//get across the line
	        addSequential(new DrivePathAction(new Move10Path60InPerSec()));
	        //lower picker
	        addSequential(new LowerPicker());
	    }
	    //set auto finished
	    addSequential(new SetAutoFinished());
    
    
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
