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
import java.util.ArrayList;

import org.usfirst.frc4089.Stealth2018.Constants;
import org.usfirst.frc4089.Stealth2018.Robot;
import org.usfirst.frc4089.Stealth2018.RobotMap;
import org.usfirst.frc4089.Stealth2018.utilities.DriveMath;
import org.usfirst.frc4089.Stealth2018.utilities.MPPoint;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class AutoFindCube extends Command {

    
	//used to keep track of the state the robot is in
	int state;
//	final int find = 1;
//	final int zero_in = 2;
//	final int move_towards = 3;
	//the variables and constants for the zero in PID loop
	int turn_last_error;
	int turn_accum_error;
	final double turn_kP = 0.003;
	final double turn_kI = 0.000075;
	final double turn_kD = 0.0000005;
	//final double kD = 0.0000001;
	//the variables and constants for the driving PID loop
	int move_last_error;
	int move_accum_error;
	final double move_kP = 0.0015;
	final double move_kI = 0.000005;
	final double move_kD = 0;
	//final double stop_kD = 0.0002;
	//the path the robot is taking to get to the block
	final ArrayList<MPPoint> encoderLogger = new ArrayList<MPPoint>();
	
    public AutoFindCube() {
        requires(Robot.drive);

    
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.logging.LogEvent("AutoFindCube Source: Commands.AutoFindCube");
    	state = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	switch (state)
    	{
    	//if the camera does not see a block
    	case 0:
    		//if it sees a cube, move to the zero in stage
        	if ((int) NetworkTable.getTable("fromPi/pixy").getDouble("pixyFrameSize", -1) > 0 )
        	{
        		state++;
        		turn_last_error = 160 - (int) NetworkTable.getTable("fromPi/pixy").getDouble("largestPixyX", -1);
        		turn_accum_error = turn_last_error;
        	}
        	//if the robot does not see a block, it simply rotates in a clockwise fashion
    		Robot.drive.DriveRobot(0, 0.25);
    		System.out.println("Looking for block...");
    		return;
    	//if the robot needs to point itself towards the block
    	case 1:
    		int blockX = (int) NetworkTable.getTable("fromPi/pixy").getDouble("largestPixyX", -1);
	    	//if the block is centered enough, start driving towards it
	    	if (blockX >= 170 && blockX <= 210)
	    	{
	    		state++;
	    		move_last_error = 220 - (int) NetworkTable.getTable("fromPi/pixy").getDouble("largestPixyWidth", -1);
	    		move_accum_error = move_last_error;
	    	}
	    	//if the block is no longer visible, start looking for it again
	    	else if ((int) NetworkTable.getTable("fromPi/pixy").getDouble("pixyFrameSize", -1) <= 0 )
	    	{
	    		state--;
	    	}
	    	System.out.println("Zeroing in...");
	    	int turn_error = blockX - 190;
	    	double turn_power = turn_error * turn_kP + turn_accum_error * turn_kI + (turn_error - turn_last_error) * turn_kD;
	    	System.out.println(turn_power);
	    	Robot.drive.DriveRobot(0, turn_power);
	    	turn_accum_error += turn_error;
	    	turn_last_error = turn_error;
	    	break;
	    //if the robot needs to move closer to the block
    	case 2:
    		blockX = (int) NetworkTable.getTable("fromPi/pixy").getDouble("largestPixyX", -1);
    		//if the block gets too off center, stop driving towards it and get it lined up again
    		if (blockX < 130 || blockX > 250)
	    	{
	    		state--;
	    	}
    		//if the block is no longer visible, start looking for it again
	    	else if ((int) NetworkTable.getTable("fromPi/pixy").getDouble("pixyFrameSize", -1) <= 0 )
	    	{
	    		state = 0;
	    	}
    		System.out.println("Driving to block...");
    		int move_error = 220 - (int) NetworkTable.getTable("fromPi/pixy").getDouble("largestPixyWidth", -1);
    		double move_power = move_error * move_kP + move_accum_error * move_kI + (move_error - move_last_error) * move_kD;
    		System.out.println(move_power);
    		turn_error = blockX - 190;
    		turn_power = turn_error * turn_kP + turn_accum_error * turn_kI + (turn_error - turn_last_error) * turn_kD;
    		System.out.println(turn_power);
    		Robot.drive.DriveRobot(move_power, turn_power);
    		move_accum_error += move_error;
    		move_last_error = move_error;
    	}
    	PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
		RobotMap.pigeonIMU.getFusedHeading(fusionStatus);
		//record path taken in order to retrace steps
		encoderLogger.add(new MPPoint(RobotMap.driveSRXDriveLF.getSelectedSensorVelocity(0), RobotMap.driveSRXDriveRF.getSelectedSensorVelocity(0), fusionStatus.heading));
    	
    }	

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	//stop driving towards block if block is close enough
        return (int) NetworkTable.getTable("fromPi/pixy").getDouble("largestPixyWidth", -1) > 200;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("Block found!");
    	//stop driving, and tell the robot the path it took to get back to where it was before
    	Robot.drive.DriveRobot(0, 0);
    	Robot.path = DriveMath.GeneratePath(encoderLogger);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
