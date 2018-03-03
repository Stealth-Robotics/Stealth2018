//----------------------------------------------------------------------------
//
//  $Workfile: Elevator.java$
//
//  $Revision: X$
//
//  Project:    Stealth Libraries
//
//                            Copyright (c) 2018
//                           Cedarcrest High School
//                            All Rights Reserved
//
//  Modification History:
//  $Log:
//  $
//
//----------------------------------------------------------------------------
package org.usfirst.frc4089.Stealth2018.subsystems;

import org.usfirst.frc4089.Stealth2018.Constants;
import org.usfirst.frc4089.Stealth2018.RobotMap;
import org.usfirst.frc4089.Stealth2018.commands.*;
import org.usfirst.frc4089.Stealth2018.utilities.DriveMath;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.command.Subsystem;

//----------------------------------------------------------------------------
//Class Declarations
//----------------------------------------------------------------------------
//
//Class Name: Elevator
//
//Purpose:
//  Moves both the Elevator and Picker Elevator up and down
//
//----------------------------------------------------------------------------
public class Elevator extends Subsystem {

  //--------------------------------------------------------------------
  // Purpose:
  //     Sets the default command 
  //
  // Notes:
  //     none
  //--------------------------------------------------------------------  
	public int lastElevatorEncoderValue = 0;
	public int elevatorEncoderValue = 0;
	
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
  //--------------------------------------------------------------------
  //--------------------------------------------------------------------
  // Main Elevator Functions
  //--------------------------------------------------------------------
  //--------------------------------------------------------------------
  //--------------------------------------------------------------------
  // --------------------------------------------------------------------
  // Constants:
  // --------------------------------------------------------------------
  
// --> Moved to Constants.java
  
  // --------------------------------------------------------------------
  // Attributes:
  // --------------------------------------------------------------------
  private int pidElevatorTarget;
  private double elevatorPreviousError = 0;
  private double elevatorIntegral = 0;
  private double elevatorLastEncoderTicks = 0;
  private double elevatorEncoderCount = 0;

  
  //--------------------------------------------------------------------
  // Purpose:
  //     Drive elevator using the joystick 
  //
  // Notes:
  //     none
  //--------------------------------------------------------------------  
  public void HandleElevator(double yElevator) {
    yElevator = DriveMath.DeadBand(yElevator, 0.25);
    int change = (int)(-yElevator * 10);
    boolean topSwitch = RobotMap.elevatorSwitchTop.get();
    boolean bottomSwitch = RobotMap.elevatorSwitchBottom.get();

    // only change things if the user wants to to advoid messing with auto
    if(0 != change)
    {
      // if nothing is pressed move the elevator
      if((false == topSwitch)&&
          (false == bottomSwitch))
      {
        AddElevatorTarget(change);
      }
      else
      {  
        // Don't change the drive if the switches are set
        if((true == bottomSwitch)&&
            (change>0))
         {
           AddElevatorTarget(change);
         }
        else
        {
            if(change<0)
           {
             AddElevatorTarget(change);
           }
        }
      }
    }
  }   
 
  //--------------------------------------------------------------------
  // Purpose:
  //     Sets the elevator to a known position 
  //
  // Notes:
  //     none
  //--------------------------------------------------------------------  
  public void SetElevatorTarget(int target)
  {
    pidElevatorTarget = target;  
  }
    
  //--------------------------------------------------------------------
  // Purpose:
  //     Adds an amount to the current position 
  //
  // Notes:
  //     none
  //--------------------------------------------------------------------  
  public void AddElevatorTarget(int amount)
  {
    pidElevatorTarget += amount;  
  }
    
  //--------------------------------------------------------------------
  // Purpose:
  //     Get Elevator Current Posiotn 
  //
  // Notes:
  //     need to check for null/ bad encoder values prior to returning a result
  //--------------------------------------------------------------------  
  public int GetElevatorPosition() {
	
	int currentElevatorEncoderValue = RobotMap.elevatorEncoder.get();
	
	if (currentElevatorEncoderValue == 0) {
		System.out.println("Elevator Encoder has Zero Value");
	} else if (currentElevatorEncoderValue == lastElevatorEncoderValue) {
		System.out.println("Encoder Value has not changes since last update");
	} else {
		elevatorEncoderValue = currentElevatorEncoderValue;
	}
	
	lastElevatorEncoderValue = currentElevatorEncoderValue;
	
    return elevatorEncoderValue;
    
   }
  //--------------------------------------------------------------------
  // Purpose:
  //     Runs the motion control on the elevator 
  //
  // Notes:
  //     none
  //--------------------------------------------------------------------  
  public void MoveElevatorToTarget() {
    //get current ticks
    //int elevatorEncoderTicks = RobotMap.elevatorEncoder.get();
	int elevatorEncoderTicks = GetElevatorPosition();
	
    //make sure it doesn't go past limit switches
    if (RobotMap.elevatorSwitchBottom.get() == true) {
    	ResetElevatorEncoder();
      
      SetElevatorTarget(50);
      elevatorIntegral = 0.0;
      elevatorPreviousError = 0.0;
    }
    
    if (RobotMap.elevatorSwitchTop.get() == true) {
      SetElevatorTarget(elevatorEncoderTicks-50);
      elevatorIntegral = 0.0;
      elevatorPreviousError = elevatorEncoderTicks;
    }
    
	  //calculate error
	  double error = pidElevatorTarget - elevatorEncoderTicks;
	  //Calculate Motor Power using PID loop
	  elevatorIntegral = elevatorIntegral + error;
	  elevatorIntegral = Math.max(0, Math.min(1440, elevatorIntegral));
	  double derivative = (error - elevatorPreviousError);
	  double motorOutput = Constants.elevatorKp*error + Constants.elevatorKi*elevatorIntegral + Constants.elevatorKd*derivative;
	  elevatorPreviousError = error;
	  //clamp the value between -1 and 1
	  motorOutput = Math.max(-1, Math.min(1, motorOutput));
	  
	  elevatorLastEncoderTicks = elevatorEncoderTicks;
	  
	  //set the motor to the correct value
	  RobotMap.elevatorMotor.set(motorOutput);

	  RobotMap.netTable.putNumber("elevatorMotorOutput", motorOutput);
    RobotMap.netTable.putNumber("elevatorError", error);
    RobotMap.netTable.putNumber("elevatorSetPoint", pidElevatorTarget);
    RobotMap.netTable.putNumber("elevatorIntegral", elevatorIntegral);
    RobotMap.netTable.putNumber("elevatorDerivative", derivative);
  }

  //--------------------------------------------------------------------
  //--------------------------------------------------------------------
  //--------------------------------------------------------------------
  // Picker Elevator Functions
  //--------------------------------------------------------------------
  //--------------------------------------------------------------------
  //--------------------------------------------------------------------

  // --------------------------------------------------------------------
  // Constants:
  // --------------------------------------------------------------------

  	// --> Moved to Constants.java

  // --------------------------------------------------------------------
  // Attributes:
  // --------------------------------------------------------------------
  private int pidPickerElevatorTarget;
  double pickerElevatorPreviousError = 0;
  double pickerElevatorIntegral = 0;
  double pickerElevatorLastEncoderTicks = 0;
  double pickerElevatorEncoderCount = 0;
  
  //--------------------------------------------------------------------
  // Purpose:
  //     Sets the setpoint from the joystick 
  //
  // Notes:
  //     none
  //--------------------------------------------------------------------  
  public void HandlePickerElevator(double yElevator) {
    yElevator = DriveMath.DeadBand(yElevator,0.25);
    int change = (int)(-yElevator * 10);

    // Don't change the drive if the switches are set
    if((change<0)&&
        (false == RobotMap.pickerElevatorSwitchBottom.get()))
     {
       AddPickerElevatorTarget(change);
     }
    if((change>0)&&
        (false == RobotMap.pickerElevatorSwitchBottom.get()))
     {
       AddPickerElevatorTarget((int)(-yElevator * 10));
     }
  }
 
  //--------------------------------------------------------------------
  // Purpose:
  //     Sets the setpoint 
  //
  // Notes:
  //     none
  //--------------------------------------------------------------------  
  public void SetPickerElevatorTarget(int target) {
    pidPickerElevatorTarget = target;
   }

  //--------------------------------------------------------------------
  // Purpose:
  //     Adds to the setpoint 
  //
  // Notes:
  //     none
  //--------------------------------------------------------------------  
  public void AddPickerElevatorTarget(int target) {
    pidPickerElevatorTarget += target;
   }

  //--------------------------------------------------------------------
  // Purpose:
  //     Picker Elevator PID loop 
  //
  // Notes:
  //     none
  //--------------------------------------------------------------------  
 public void MovePickerElevatorToTarget() {
   //get current ticks
   int pickerElevatorEncoderTicks = RobotMap.pickerElevatorEncoder.get();
   boolean pickerElevatorSwitchTotalBottom = RobotMap.pickerElevatorTotalBottom.get();

/*
   if (RobotMap.pickerElevatorSwitchBottom.get() == true) {
     RobotMap.pickerElevatorEncoder.reset();
     SetPickerElevatorTarget(0);
     pickerElevatorIntegral = 0.0;
     pickerElevatorPreviousError = 0.0;
   }
   
   if (RobotMap.elevatorSwitchTop.get() == true) {
     SetPickerElevatorTarget(pickerElevatorEncoderTicks);
     pickerElevatorIntegral = 0.0;
     pickerElevatorPreviousError = pickerElevatorEncoderTicks;
   }
	*/
   
	  //calculate error
	  double error = pidPickerElevatorTarget - pickerElevatorEncoderTicks;
	  //calculate motor output power
	  pickerElevatorIntegral = pickerElevatorIntegral + error;
	  pickerElevatorIntegral = Math.max(0, Math.min(1440, pickerElevatorIntegral));
	  double derivative = (error - pickerElevatorPreviousError);
	  double motorOutput = Constants.pickerElevatorKp*error + Constants.pickerElevatorKi*pickerElevatorIntegral + Constants.pickerElevatorKd*derivative;
	  pickerElevatorPreviousError = error;
	  //clamp the value between -1 and 1
	  motorOutput = Math.max(-1, Math.min(1, motorOutput));
	  
	  pickerElevatorLastEncoderTicks = pickerElevatorEncoderTicks;
	  
	  //set the motor to the correct value
	  MoveElevator(motorOutput);
	  
	  RobotMap.netTable.putNumber("pickerElevatorMotorOutput", motorOutput);
    RobotMap.netTable.putNumber("pickerError", error);
    RobotMap.netTable.putNumber("pickerSetPoint", pidPickerElevatorTarget);
    RobotMap.netTable.putNumber("pickerElevatorIntegral", pickerElevatorIntegral);
    RobotMap.netTable.putNumber("pickerElevatorDerivative", derivative);
 }

 //--------------------------------------------------------------------
 //--------------------------------------------------------------------
 //--------------------------------------------------------------------
 // Climb Hook Functions
 //--------------------------------------------------------------------
 //--------------------------------------------------------------------
 //--------------------------------------------------------------------
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
  /**
   * @param none
   * Resets the elevator encoder
   */
  public void ResetElevatorEncoder() {
	  RobotMap.elevatorEncoder.reset();
  }
  
  /**
   * Actuates the Elevator given the following parameter
   * @param motorOutput
   */
  public void MoveElevator (double motorOutput) {
	  RobotMap.elevatorMotor.set(motorOutput);
  }
  
  public boolean GetElevatorTopSwitch() {
	  return RobotMap.elevatorSwitchTop.get();
	   
  }
  public boolean GetElevatorBottomSwitch() {
	  return RobotMap.elevatorSwitchBottom.get();
  }
}

