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
import org.usfirst.frc4089.Stealth2018.commands.DrivePathAction;
import org.usfirst.frc4089.Stealth2018.commands.HugBlock;
import org.usfirst.frc4089.Stealth2018.commands.LowerPicker;
import org.usfirst.frc4089.Stealth2018.commands.RaiseMainToTop;
import org.usfirst.frc4089.Stealth2018.commands.RaisePickerToTop;
import org.usfirst.frc4089.Stealth2018.commands.RejectBlock;
import org.usfirst.frc4089.Stealth2018.commands.SetAutoFinished;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.*;

/**
 *
 */
public class MoveForward extends CommandGroup {
    public MoveForward() {
      
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    	Robot.logging.LogEvent("MoveForward Source: autoCommands.MoveForward");
		//System.out.println("Position One Source: Commands.PositionOne");
		RobotMap.pigeonIMU.setFusedHeading(0, 30);
		addSequential(new SetAutoFinished(false));
	    //hug block
	    addSequential(new HugBlock());
	    
    	//lower picker
	    addSequential(new LowerPicker());
	    
	    //move forward
	    addSequential(new DrivePathAction(new Move10Path60InPerSec()));
      
	    //drop it
	    //addSequential(new RejectBlock());
	    
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
