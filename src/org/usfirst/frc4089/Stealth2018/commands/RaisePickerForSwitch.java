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

import edu.wpi.first.wpilibj.command.Command;

public class RaisePickerForSwitch extends Command {
    final int SWITCH_HEIGHT = 800;
  
    public RaisePickerForSwitch() {
    }

    // Called just before this Command runs the first time
    protected void initialize() {
      Robot.elevator.SetElevatorTarget(SWITCH_HEIGHT);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (Robot.elevator.GetElevatorPosition()>-SWITCH_HEIGHT);
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}