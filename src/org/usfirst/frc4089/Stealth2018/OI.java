// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc4089.Stealth2018;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;

import org.usfirst.frc4089.Stealth2018.commands.*;
import org.usfirst.frc4089.Stealth2018.subsystems.*;

import org.usfirst.frc4089.Stealth2018.commands.*;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public JoystickButton jB1A;
    public JoystickButton jB2B;
    public JoystickButton grabBlockButton;
    public JoystickButton grabClimberButton;
    public JoystickButton lowerPicker;
    public JoystickButton raisePicker;
    public JoystickButton grabClimber;
    public JoystickButton ungrabClimber;
    public JoystickButton climb;
    public JoystickButton reverseClimbMotor;
    public JoystickButton autoFindCube;
    
    public JoystickButton RotatePickerRaiseMotor;
    public JoystickButton RotatePickerRaiseMotorReverse;
    
    public JoystickButton overrideElevator;
    public JoystickButton overridePickerElevator;

    
    public Joystick driveJoystick;
    public Joystick mechJoystick;
    public Joystick dsJoystick;

    public OI() {
      mechJoystick = new Joystick(1);
      driveJoystick = new Joystick(0);
      dsJoystick = new Joystick(2);
      
      grabBlockButton = new JoystickButton(mechJoystick, 5);
      grabBlockButton.whenPressed(new RejectBlock());
      grabBlockButton.whenReleased(new HugBlock());
      
      RotatePickerRaiseMotor = new JoystickButton(mechJoystick, 4);
      RotatePickerRaiseMotor.whileHeld(new RotatePickerRaiseMotor());
      
      RotatePickerRaiseMotorReverse = new JoystickButton(mechJoystick, 3);
      RotatePickerRaiseMotorReverse.whileHeld(new RotatePickerRaiseMotorReverse());
      
      grabClimber = new JoystickButton(dsJoystick, 4);
      grabClimber.whenPressed(new GrabClimber());
      
      ungrabClimber = new JoystickButton(dsJoystick, 8);
      ungrabClimber.whenPressed(new UnGrabClimber());
      
      climb = new JoystickButton(dsJoystick, 7);
      climb.whileHeld(new RotateClimberMotor());
      
      reverseClimbMotor = new JoystickButton(dsJoystick, 3);
      reverseClimbMotor.whileHeld(new RotateClimberMotorReverse());
      
      lowerPicker = new JoystickButton(mechJoystick, 1);
      lowerPicker.whenPressed(new LowerPicker());
      
      raisePicker = new JoystickButton(mechJoystick, 2);
      raisePicker.whenPressed(new RejectBlock());
      raisePicker.whenReleased(new RaisePicker());
      
      overrideElevator = new JoystickButton(dsJoystick, 5);
      overrideElevator.whenPressed(new OverrideElevator());
      
      overridePickerElevator = new JoystickButton(dsJoystick, 1);
      overridePickerElevator.whenPressed(new OverridePickerElevator());
      
      
      
      if (Constants.UsePixyAutoFindCube) {
    	  autoFindCube = new JoystickButton(driveJoystick, 4);
    	  autoFindCube.whenReleased(new AutoFindCube());
      }

    }
}

