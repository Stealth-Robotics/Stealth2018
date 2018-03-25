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
import edu.wpi.first.wpilibj.Joystick;

/**
 *
 */
public class ShootBlock extends Command {
  
    public ShootBlock() {
        requires(Robot.picker);
        //System.out.println("RejectBlock");
    }

    protected void initialize() {
      //System.out.println("Init");
    }

    protected void execute() {
      System.out.println("Close picker");
    	Robot.picker.setPickerMotors(1);;
    }

    protected boolean isFinished() {
        return true;
    }

    protected void end() {
    	//Robot.picker.set
    }

    protected void interrupted() {
      end();
    }
}
