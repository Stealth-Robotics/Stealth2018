package org.usfirst.frc4089.Stealth2018.commands;

import org.usfirst.frc4089.Stealth2018.Constants;
import org.usfirst.frc4089.Stealth2018.Robot;


import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoMoveLift extends Command {
	
	private double errorAvg;


    public AutoMoveLift(double angle, double radius) {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.drive);
    	
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
