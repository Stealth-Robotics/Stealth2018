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

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.print.DocFlavor.STRING;

import org.usfirst.frc4089.Stealth2018.MPPaths.Path;
import org.usfirst.frc4089.Stealth2018.autoCommands.*;
import org.usfirst.frc4089.Stealth2018.commands.*;
import org.usfirst.frc4089.Stealth2018.subsystems.*;

import com.ctre.phoenix.sensors.PigeonIMU;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in 
 * the project.
 */
public class Robot extends TimedRobot {
    
    SendableChooser<Command> chooser = new SendableChooser<>();
    
    public static String CurrentMode;
    
    public static OI oi;
    public static Logging logging;

    public static Drive drive;
    public static Elevator elevator;
    public static Picker picker;
    public static Utilities utilities;
    public static Climb climb;

    public static Path path;
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
    	CurrentMode = "Initing";
    	
        RobotMap.init();
        RobotMap.pigeonIMU.setFusedHeading(0, 30);
        
        if(Constants.UseCamera) {
	        //start camera server
	        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
	        camera.setResolution(320,240);
	        camera.setFPS(30);
        }
        
        logging = new Logging();
        
        drive = new Drive();
        elevator = new Elevator();
        picker = new Picker();
        utilities = new Utilities();
        climb = new Climb();

        Robot.elevator.SetElevatorTarget(0);
        Robot.elevator.SetPickerElevatorTarget(0);
        RobotMap.elevatorMotor.setSelectedSensorPosition(0, 0, 20);
        RobotMap.pickerElevatorMotor.setSelectedSensorPosition(0, 0, 20);
        
        
        // OI must be constructed after subsystems. If the OI creates Commands
        //(which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
        oi = new OI();
        
        chooser.addObject("Literaly Just Move Forward", new MoveForward());
        chooser.addObject("Position 1 Switch From Front", new Position1Path1());
        chooser.addObject("Position 1 Scale From Front", new Position1Path2());
        chooser.addObject("Position 1 Scale From Side", new Position1Path3());
        chooser.addObject("Position 1 Cross Scale From Side or Scale From Front", new Position1Path4());
        chooser.addDefault("Position 3 Switch From Side", new Position3Path1());
        chooser.addObject("Position 5 Switch From Front", new Position5Path1());
        chooser.addObject("Position 5 Scale From Front", new Position5Path2());
        chooser.addObject("Position 5 Scale From Side", new Position5Path3());
        chooser.addObject("Position 5 Cross Scale From Side or Scale From Front", new Position5Path4());
        SmartDashboard.putData("Auto mode", chooser);
        
        System.out.println("robot init");
        
    }


    @Override
    public void robotPeriodic(){
    }
    
    @Override
    public void disabledInit(){
    	
    	CurrentMode = "Disabled";
      if (mAutoCommand != null) mAutoCommand.cancel();
      
      
    }

    private void DisplaySensors()
    {
    	PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
    	RobotMap.pigeonIMU.getFusedHeading(fusionStatus);
      System.out.format("ELEVATOR TOP: %b ELEVATOR BOTTOM: %b PICKER TOP: %b PICKER BOTTOM: %b ELEVATOR TICKS: %d PICKER TICKS: %d HEADING: %f\n", 
          RobotMap.elevatorSensors.isFwdLimitSwitchClosed(),
          RobotMap.elevatorSensors.isRevLimitSwitchClosed(),
          RobotMap.pickerElevatorSensors.isFwdLimitSwitchClosed(),
          RobotMap.pickerElevatorSensors.isRevLimitSwitchClosed(),
          RobotMap.elevatorMotor.getSelectedSensorPosition(0),
          RobotMap.pickerElevatorMotor.getSelectedSensorPosition(0),
      	  fusionStatus.heading
          );

    }
    
    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
        
        RobotMap.pigeonIMU.setFusedHeading(0, 30);
        
        if(Robot.oi.mechJoystick.getRawButton(9))
        {
        	DisplaySensors();
        }
    }

    Command mAutoCommand;
    
    @Override
    public void autonomousInit() {
    	CurrentMode = "Autonomus";
      if (mAutoCommand != null) mAutoCommand.cancel();
      Robot.climb.ungrabClimber();
      
      RobotMap.SetUpTalonsForAuto();
      drive.ClearCurrentAngle();
      RobotMap.pigeonIMU.setFusedHeading(0.0, 30);

      drive.SetAuto();
      
      Robot.elevator.SetElevatorTarget(0);
      Robot.elevator.SetPickerElevatorTarget(0);
      
      if(chooser.getSelected().getName().equals("MoveForward")) {
    	  mAutoCommand = new MoveForward();
          Scheduler.getInstance().add(mAutoCommand);
      }
      if(chooser.getSelected().getName().equals("Position1Path1"))
      {
        mAutoCommand = new Position1Path1();
        Scheduler.getInstance().add(mAutoCommand);
      }
      else if(chooser.getSelected().getName().equals("Position1Path2"))
      {
        mAutoCommand = new Position1Path2();
        Scheduler.getInstance().add(mAutoCommand);
      }
      else if(chooser.getSelected().getName().equals("Position1Path3"))
      {
        mAutoCommand = new Position1Path3();
        Scheduler.getInstance().add(mAutoCommand);
      }
      else if(chooser.getSelected().getName().equals("Position1Path4"))
      {
        mAutoCommand = new Position1Path4();
        Scheduler.getInstance().add(mAutoCommand);
      }
      else if(chooser.getSelected().getName().equals("Position3Path1"))
      {
        mAutoCommand = new Position3Path1();
        Scheduler.getInstance().add(mAutoCommand);
      }
      else if(chooser.getSelected().getName().equals("Position5Path1"))
      {
        mAutoCommand = new Position5Path1();
        Scheduler.getInstance().add(mAutoCommand);
      }
      else if(chooser.getSelected().getName().equals("Position5Path2"))
      {
        mAutoCommand = new Position5Path2();
        Scheduler.getInstance().add(mAutoCommand);
      }
      else if(chooser.getSelected().getName().equals("Position5Path3"))
      {
        mAutoCommand = new Position5Path3();
        Scheduler.getInstance().add(mAutoCommand);
      }
      else if(chooser.getSelected().getName().equals("Position5Path4"))
      {
        mAutoCommand = new Position5Path4();
        Scheduler.getInstance().add(mAutoCommand);
      }
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
    	logging.Log();
       Scheduler.getInstance().run();
       
       if(!RobotMap.overrideElevator) {
           Robot.elevator.MoveElevatorToTarget();
       }
       if(!RobotMap.overridePickerElevator) {
           Robot.elevator.MovePickerElevatorToTarget();
       }
    }

    @Override
    public void teleopInit() {
    	CurrentMode = "Teleop";
        //stop auto
        if (mAutoCommand != null) mAutoCommand.cancel();
        
        //Init teleop
        System.out.println("tele init");
        RobotMap.SetUpTalonsForTele();
        Robot.drive.SetTele();
        Robot.climb.ungrabClimber();
        Robot.picker.hugBlock();
        RobotMap.utilitiesPCMCompressor.setClosedLoopControl(true);
        //set the elevator targets to be where they are
        Robot.elevator.SetElevatorTarget(RobotMap.elevatorMotor.getSelectedSensorPosition(0));
        Robot.elevator.SetPickerElevatorTarget(RobotMap.pickerElevatorMotor.getSelectedSensorPosition(0));
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
    	logging.Log();
        Scheduler.getInstance().run();
        
        //DisplaySensors();
        
        Robot.drive.DriveRobot(oi.driveJoystick);
        Robot.elevator.DriveElevator(oi.mechJoystick);
        Robot.picker.DrivePickerWheels(oi.mechJoystick);
        
        if(!RobotMap.overrideElevator) {
        	Robot.elevator.MoveElevatorToTarget();
        }
        if(!RobotMap.overridePickerElevator) {
        	Robot.elevator.MovePickerElevatorToTarget();
        }
    }
    
    
    @Override
    public void testInit() {
    	CurrentMode = "Test";
    }   

    @Override
    public void testPeriodic() {
    	logging.Log();
    }   
}
