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
import org.usfirst.frc4089.Stealth2018.Robot;
import org.usfirst.frc4089.Stealth2018.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RotateClimberMotor extends Command {

    
    public RotateClimberMotor() {

    
        requires(Robot.picker);

    
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
      RobotMap.climbMotor.set(0.2);
    }

    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    	RobotMap.climbMotor.set(0);
    }

    protected void interrupted() {
      RobotMap.climbMotor.set(0);
      end();
    }
}
