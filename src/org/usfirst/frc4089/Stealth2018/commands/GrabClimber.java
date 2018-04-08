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
import org.usfirst.frc4089.Stealth2018.*;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GrabClimber extends Command {

    
    public GrabClimber() {
        requires(Robot.climb);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.logging.LogEvent("GrabClimber Source: Commands.GrabClimber");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
      Robot.climb.grabClimber();
    }

    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    protected void interrupted() {
    	end();
    }
}
