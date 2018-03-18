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

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;


/**
 *
 */
public class Picker extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
    }
    
    public void rejectBlock () {
      RobotMap.pickerArms.set(true);
      //System.out.println("Open Arms Source: Picker.rejectBlock()");
    }
    
    public void hugBlock () {
      RobotMap.pickerArms.set(false);
      
      //System.out.println("Close Arms Source: Picker.hugBlock()");
    }


    public void setRaisePickerMotor(double value) {
      RobotMap.pickerRaiseMotor.set(value);
      
      //System.out.println("Set Picker Raise Motor " + value + " Source: Picker.setRaisePickerMotor()");
    }
    
    public void stopRaisePickerMotor() {
    	RobotMap.pickerRaiseMotor.set(0);
    	
    	//System.out.println("Stop Picker Raise Motor Source: Picker.setRaisePickerMotor()");
    }
    
    public void setPickerMotors(double value) {
    	//pickerLeftMotor is negative because it is the inverse of pickerRightMotor
    	RobotMap.pickerLeftMotor.set(-value);
    	RobotMap.pickerRightMotor.set(value);
    	
    	//System.out.println("Set Picker Motors " + value + " Source: Picker.setPickerMotors()");
    }
    
    public void stopPickerMotors() {
    	RobotMap.pickerLeftMotor.set(0);
    	RobotMap.pickerRightMotor.set(0);
    	
    	//System.out.println("Stop Picker Motors Source: Picker.stopPickerMotors()");
    }
    
    public boolean getPickerPositionSwitch() {
    	return RobotMap.SwitchPickerBottom.get();
    }
    
    public void DrivePickerWheels(Joystick mechJoystick) {
    	if (mechJoystick.getRawAxis(3) > 0.1) {
    		setPickerMotors(mechJoystick.getRawAxis(3));
          } else if (mechJoystick.getRawAxis(2) > 0.1) {
        	  setPickerMotors(-mechJoystick.getRawAxis(2));
          } else {
            setPickerMotors(-0.4);
          }
    }
}

