// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc4089.Stealth2018.subsystems;

import org.usfirst.frc4089.Stealth2018.Constants;
import org.usfirst.frc4089.Stealth2018.RobotMap;
import org.usfirst.frc4089.Stealth2018.commands.*;
import org.usfirst.frc4089.Stealth2018.subsystems.Drive.DriveControlState;
import org.usfirst.frc4089.Stealth2018.utilities.DriveMath;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.*;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Elevator extends Subsystem {
    private int elevatorTargetTick;
    private int pickerElevatorTargetTick;
  	private int pidElevatorTarget;
  	private int pidPickerElevatorTarget;

	
    public void initDefaultCommand() {
      setDefaultCommand(new UserElevator());
    }
    
    //--------------------------------------------------------------------
    // Purpose:
    //     Drive using the joystick 
    //
    // Notes:
    //     none
    //--------------------------------------------------------------------  
    public void DriveElevator(Joystick driveJoystick) {
      HandleElevator(driveJoystick.getRawAxis(1));
      HandlePickerElevator(driveJoystick.getRawAxis(5));
    }
    
      //--------------------------------------------------------------------
      // Purpose:
      //     Drive using the joystick 
      //
      // Notes:
      //     none
      //--------------------------------------------------------------------  
    public void HandleElevator(double yElevator) {
	  boolean elevatorSwitchTop = RobotMap.elevatorSwitchTop.get();
	  boolean elevatorSwitchBottom = RobotMap.elevatorSwitchBottom.get();
		  
	  yElevator = DriveMath.DeadBand(yElevator, 0.25);
	
	  if(true == elevatorSwitchBottom)
	  {
	    // If the elevator is at the bottom and we want to go down, don't
	    if(yElevator > 0)
	    {
	      yElevator = 0;
	    }
	  }
	  else
	  {
	    if(true == elevatorSwitchTop)
	    {
	      if(yElevator < 0)
	      {
	        yElevator = 0;
	      }
	    }
	  }
	  
	  elevatorTargetTick += yElevator * 10;
	  	 
	  SetElevatorTarget(elevatorTargetTick);
    }   
 
      //--------------------------------------------------------------------
      // Purpose:
      //     Drive using the joystick 
      //
      // Notes:
      //     none
      //--------------------------------------------------------------------  
    public void HandlePickerElevator(double yElevator) {
    	boolean elevatorSwitchTop = RobotMap.pickerElevatorSwitchTop.get();
        boolean elevatorSwitchBottom = RobotMap.pickerElevatorSwitchBottom.get();
      
      yElevator = DriveMath.DeadBand(yElevator,0.25);

      if(true == elevatorSwitchBottom)
      {
        if(yElevator > 0)
        {
          yElevator = 0;
        }
      }
      else
      {
        if(true == elevatorSwitchTop)
        {
          if(yElevator < 0)
          {
            yElevator = 0;
          }
        }
      }

	  pickerElevatorTargetTick += yElevator * 10;

	  SetPickerElevatorTarget(pickerElevatorTargetTick);
    }
  

  	public void SetElevatorTarget(int target) {
		pidElevatorTarget = target;
	}
  
  	
  final double elevatorKPconst = 0.01;
  public void MoveElevatorToTarget() {
	  //get limit switches
	  boolean elevatorSwitchTop = RobotMap.elevatorSwitchTop.get();
      boolean elevatorSwitchBottom = RobotMap.elevatorSwitchBottom.get();
      //reset encoder if at bottom switch
	  if (elevatorSwitchBottom == true) {
    	  RobotMap.elevatorEncoder.reset();
      }
	  //get current ticks
	  int elevatorEncoderTicks = RobotMap.elevatorEncoder.get();
	  //calculate error
	  double error = pidElevatorTarget - elevatorEncoderTicks;
	  //set the motor power
	  double motorOutput = error * elevatorKPconst;
	  //clamp the value between -1 and 1
	  motorOutput = Math.max(-1, Math.min(1, motorOutput));
	  //make sure it doesn't go past limit switches
	  if (elevatorSwitchBottom == true && motorOutput < 0) {
		  motorOutput = 0;
	  }
	  
	  if (elevatorSwitchTop == true && motorOutput > 0) {
		  motorOutput = 0;
	  }
	  //set the motor to the correct value
	  RobotMap.elevatorMotor.set(motorOutput);
	  //Debugin stuff
	  System.out.println("elevatorEncoderTicks: " + elevatorEncoderTicks);
	  System.out.println("elevatorMotorPower: " + RobotMap.elevatorMotor.get());
 }
 
 public void SetPickerElevatorTarget(int target) {
 	pidPickerElevatorTarget = target;
 }

 final double pickerElevatorKPconst = 0.01;
 public void MovePickerElevatorToTarget() {
	 //get limit switches
	 boolean elevatorSwitchTop = RobotMap.pickerElevatorSwitchTop.get();
     boolean elevatorSwitchBottom = RobotMap.pickerElevatorSwitchBottom.get();
     //reset encoder if at bottom switch
	  if (elevatorSwitchBottom == true) {
   	  RobotMap.pickerElevatorEncoder.reset();
     }
	  //get current ticks
	  int pickerElevatorEncoderTicks = RobotMap.pickerElevatorEncoder.get();
	  //calculate error
	  double error = pidPickerElevatorTarget - pickerElevatorEncoderTicks;
	  //set the motor power
	  double motorOutput = error * pickerElevatorKPconst;
	  //clamp the value between -1 and 1
	  motorOutput = Math.max(-1, Math.min(1, motorOutput));
	  //make sure it doesn't go past limit switches
	  if (elevatorSwitchBottom == true && motorOutput < 0) {
		  motorOutput = 0;
	  }
	  
	  if (elevatorSwitchTop == true && motorOutput > 0) {
		  motorOutput = 0;
	  }
	  //set the motor to the correct value
	  RobotMap.pickerElevatorMotor.set(motorOutput);
	  //Debugin stuff
	  System.out.println("pickerElevatorEncoderTicks: " + pickerElevatorEncoderTicks);
	  System.out.println("pickerElevatorMotorPower: " + RobotMap.pickerElevatorMotor.get());
	 
	 /*boolean elevatorSwitchTop = RobotMap.pickerElevatorSwitchTop.get();
     boolean elevatorSwitchBottom = RobotMap.pickerElevatorSwitchBottom.get();
     
     if (elevatorSwitchBottom == true) {
    	 RobotMap.pickerElevatorEncoder.reset();
     }
	 
	 // when -100000, lower until bottom limit 2 == true
	 
	  int pickerElevatorEncoderTicks = RobotMap.pickerElevatorEncoder.get();
	  if(pidPickerElevatorTarget == pickerElevatorEncoderTicks)
	  {
		  RobotMap.pickerElevatorMotor.set(0);
	  }
	  if(pidPickerElevatorTarget < pickerElevatorEncoderTicks)
	  {
		  RobotMap.pickerElevatorMotor.set(1);
	  }
	  else if(pidPickerElevatorTarget > pickerElevatorEncoderTicks)
	  {
		  RobotMap.pickerElevatorMotor.set(-1);
	  }
	  
	  System.out.println("pickerElevatorEncoderTicks: " + pickerElevatorEncoderTicks);*/
 } 
   
 //TODO: Move to it's own command not under the elevator subsystem
  public void EngageClimbHook() {
	  //1. Lower elevator to bottom (limit switch 1 = true)
	  SetElevatorTarget(0);
	  SetPickerElevatorTarget(0);
	  
	  //2. Engage climb hook pneumatic 
	  
	  
	  //3. Lower elevator to retract grabber
	  SetPickerElevatorTarget(-100000); 
	  
	  //4. Engage pneumatic to lock grabber in upright position
	  
	  
	  //5. Return to user drive mode
	  
	  
  }
}

