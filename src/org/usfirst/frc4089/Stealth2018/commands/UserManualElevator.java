package org.usfirst.frc4089.Stealth2018.commands;

import org.usfirst.frc4089.Stealth2018.Constants;
import org.usfirst.frc4089.Stealth2018.Robot;
import org.usfirst.frc4089.Stealth2018.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class UserManualElevator extends Command {


    public UserManualElevator() {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.elevator);
    	
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	//Robot.elevator.DriveElevator( driveJoystick);
    	if (Robot.oi.mechJoystick.getRawAxis(1) > 0) {
    		Robot.elevator.MoveElevator(1);
    	} else if (Robot.oi.mechJoystick.getRawAxis(1) < 0) {
    		Robot.elevator.MoveElevator(-1);
    	} else {
    		Robot.elevator.MoveElevator(0);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	
    	return RobotMap.elevatorSwitchTop.get() || RobotMap.elevatorSwitchBottom.get();
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
