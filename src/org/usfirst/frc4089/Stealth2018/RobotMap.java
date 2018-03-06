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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    public static WPI_TalonSRX driveSRXDriveLR;
    public static WPI_TalonSRX driveSRXDriveLF;
    public static WPI_TalonSRX driveSRXDriveRR;
    public static WPI_TalonSRX driveSRXDriveRF;
    
    public static WPI_TalonSRX pickerSRXPickerL;
    public static WPI_TalonSRX pickerSRXPickerR;
    
    public static DoubleSolenoid pickerDoubleSolenoid1Pick;
    public static Compressor utilitiesPCMCompressor;
    
    // Elevator
    public static Encoder elevatorEncoder;
    public static DigitalInput elevatorSwitchTop;
    public static DigitalInput elevatorSwitchBottom;
    public static WPI_TalonSRX elevatorMotor;

    // Picker
    public static Encoder pickerElevatorEncoder;
    public static DigitalInput pickerElevatorSwitchTop;
    public static DigitalInput pickerElevatorSwitchBottom;
    public static DigitalInput pickerElevatorTotalBottom;
    public static WPI_TalonSRX pickerElevatorMotor;
    public static WPI_TalonSRX pickerRaiseMotor;
    
    public static WPI_TalonSRX pickerLeftMotor;
    public static WPI_TalonSRX pickerRightMotor;
    
    public static Solenoid pickerArms;
    public static Solenoid climberGrabber;
    
    public static PigeonIMU pigeonIMU;

    public static NetworkTable netTable;
    
    public static boolean overrideElevator;
    public static boolean overridePickerElevator;
    
    
    public static WPI_TalonSRX climbMotor;
    
    
    public static void init() {
      driveSRXDriveLR = new WPI_TalonSRX(Constants.CANTalonSRXDriveLR);
      driveSRXDriveLF = new WPI_TalonSRX(Constants.CANTalonSRXDriveLF);
      driveSRXDriveRR = new WPI_TalonSRX(Constants.CANTalonSRXDriveRR);
      driveSRXDriveRF = new WPI_TalonSRX(Constants.CANTalonSRXDriveRF);

      pickerSRXPickerL = new WPI_TalonSRX(Constants.CANTalonSRXPickerL);
      pickerSRXPickerR = new WPI_TalonSRX(Constants.CANTalonSRXPickerR);
      
      //pickerDoubleSolenoid1Pick = new DoubleSolenoid(0, 0, 1);
//        utilitiesPowerDistributionPanel1 = new PowerDistributionPanel(18);
      utilitiesPCMCompressor = new Compressor(16);
      utilitiesPCMCompressor.setClosedLoopControl(true);

      elevatorSwitchTop  = new DigitalInput(9);
      elevatorSwitchBottom = new DigitalInput(8);
      elevatorEncoder  = new Encoder(0, 1, true);
      elevatorMotor = new WPI_TalonSRX(Constants.CANTalonSRXElevator);
      elevatorMotor.setInverted(true);
       
      pickerElevatorSwitchTop = new DigitalInput(7);
      pickerElevatorSwitchBottom = new DigitalInput(5);
      pickerElevatorTotalBottom = new DigitalInput(6);
      pickerElevatorEncoder  = new Encoder(2, 3, false);
      pickerElevatorMotor = new WPI_TalonSRX(Constants.CANTalonSRXClimb);
      
      pickerRaiseMotor = new WPI_TalonSRX(9);
      
      pickerLeftMotor = new WPI_TalonSRX(Constants.CANTalonSRXPickerL);
      pickerRightMotor = new WPI_TalonSRX(Constants.CANTalonSRXPickerR);
      
      pickerArms = new Solenoid(16,5);
      climberGrabber = new Solenoid(16,7);

      pigeonIMU = new PigeonIMU(driveSRXDriveLR);
      pigeonIMU.setFusedHeading(0.0, 10);
      netTable = NetworkTable.getTable("FRCRobot");
      
      overrideElevator = true;
      overridePickerElevator = true;
      
      climbMotor = new WPI_TalonSRX(Constants.CANTalonSRXClimb);
    }
    
    
    public static void SetUpTalonsForTele()
    {
      SetUpTalonForTele(driveSRXDriveLF);
      SetUpTalonForTele(driveSRXDriveRF);
      driveSRXDriveLR.set(ControlMode.Follower, Constants.CANTalonSRXDriveLF);
      driveSRXDriveRR.set(ControlMode.Follower, Constants.CANTalonSRXDriveRF);
      driveSRXDriveRF.setInverted(true);
      driveSRXDriveRR.setInverted(true);
      
    }
    
    private static void SetUpTalonForTele(WPI_TalonSRX talon) {
      talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
      talon.setSensorPhase(true);

      /* set the peak and nominal outputs, 12V means full */
      talon.configNominalOutputForward(0, Constants.kTimeoutMs);
      talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
      talon.configPeakOutputForward(1, Constants.kTimeoutMs);
      talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

      /* set closed loop gains in slot0 */
      talon.config_kF(Constants.kPIDLoopIdx, 0.34, Constants.kTimeoutMs);
      talon.config_kP(Constants.kPIDLoopIdx, 0.2, Constants.kTimeoutMs);
      talon.config_kI(Constants.kPIDLoopIdx, 0, Constants.kTimeoutMs);
      talon.config_kD(Constants.kPIDLoopIdx, 0, Constants.kTimeoutMs);

      talon.setSafetyEnabled(true);
      talon.setExpiration(Constants.kTimeoutMs);
    }

    public static void SetUpTalonsForAuto()
    {
      SetUpTalonForAuto(driveSRXDriveLF);
      SetUpTalonForAuto(driveSRXDriveRF);
      driveSRXDriveLR.set(ControlMode.Follower, Constants.CANTalonSRXDriveLF);
      driveSRXDriveRR.set(ControlMode.Follower, Constants.CANTalonSRXDriveRF);
      driveSRXDriveRF.setInverted(true);
      driveSRXDriveRR.setInverted(true);
    }
    
    private static void SetUpTalonForAuto(WPI_TalonSRX talon) {
      talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
      talon.setSensorPhase(true); /* keep sensor and motor in phase */
      talon.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

      talon.config_kF(0, 0.076, Constants.kTimeoutMs);
      talon.config_kP(0, 2.000, Constants.kTimeoutMs);
      talon.config_kI(0, 0.0, Constants.kTimeoutMs);
      talon.config_kD(0,20.0, Constants.kTimeoutMs);

      //talon.configMotionProfileTrajectoryPeriod(20, Constants.kTimeoutMs); //Our profile uses 10 ms timing
      /* status 10 provides the trajectory target for motion profile AND motion magic */
      //talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 20, Constants.kTimeoutMs);
      
    }
   
}
