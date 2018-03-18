package org.usfirst.frc4089.Stealth2018.commands;

import org.usfirst.frc4089.Stealth2018.Constants;
import org.usfirst.frc4089.Stealth2018.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoClimb extends CommandGroup {
	    
	    public  AutoClimb() {
	    	System.out.println("AutoClimb");
	    	requires(Robot.climb);
	        // Add Commands here:
	        // e.g. addSequential(new Command1());
	        //      addSequential(new Command2());
	        // these will run in order.

	        // To run multiple commands at the same time,
	        // use addParallel()
	        // e.g. addParallel(new Command1());
	        //      addSequential(new Command2());
	        // Command1 and Command2 will run in parallel.

	        // A command group will require all of the subsystems that each member
	        // would require.
	        // e.g. if Command1 requires chassis, and Command2 requires arm,
	        // a CommandGroup containing them would require both the chassis and the
	        // arm.
	    	
	    	//set distance + orientation
	    	//lower elevator
	    	//lower pickerelevator
	    	//lower picker
	    	//actuate climb hook
	    	//raise picker
	    	//raise elevator + pickerelevator
	    	//drive forward
	    	//lower elevator + pickerelevator
	    	//run winch
	    	
	    	//TODO // resolve assumtion distance + orientation is set
	    	addSequential(new AutoElevator(0,0));
	    	//commented out lower picker for now because it is going to be down when we call this command
	    	//addSequential(new LowerPicker());
	    	addSequential(new IsPickerInPositionForClimb());
	    	
	    	addSequential(new GrabClimber());
	    	addSequential(new RaisePicker());
	    	addParallel(new RejectBlock());
	    	addSequential(new AutoElevator(1400,1400));
	    	//TODO //drive forward to position
	    	addSequential(new AutoElevator(0,0));
	    	addSequential(new RaiseClimber());
	    	
	    	  	
	    	
	    	System.out.println("AutoClimb - Complete");
	    }
}



