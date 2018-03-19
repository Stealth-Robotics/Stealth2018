package org.usfirst.frc4089.Stealth2018;

//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Constants {
  
	public static final boolean UseCamera = true;
	public static final boolean UsePixyAutoFindCube = true;
	
	//below are values for 2018
	public static final double pickerLRSpeed = 1;
	public static final double pickerLFSpeed = 1;
	public static final double pickerRRSpeed = 1;
	public static final double pickerRFSpeed = 1;
	public static final double pickerLeftSpeed = 1;
	public static final double pickerRightSpeed = 1;
	
	// Drive Assignments
	public static final int CANTalonSRXDriveLR = 3;
	public static final int CANTalonSRXDriveLF = 1;
	public static final int CANTalonSRXDriveRR = 4;
	public static final int CANTalonSRXDriveRF = 2;
	
	// Elevator Picker Assignments 
    public static final int CANTalonSRXElevator = 5;
    public static final int CANTalonSRXPickerElevator = 7;
    public static final int CANTalonSRXPickerRaise = 10;
	public static final int CANTalonSRXPickerL = 6;
	public static final int CANTalonSRXPickerR = 8;
	public static final int CANTalonSRXClimb = 9;
	
	public static final int CANPCM1 = 16;
	public static final int CANPCM2 = 17;
	public static final int CANPDP = 18;

	// User Interface consts
	public static final int kFastButton = 1;
	public static final int kSlowButton = 2;
	public static final int kForwardAxes = 1;
	public static final int kMainTurnAxes = 4;
	public static final int kBackupTurnAxes = 2;
	
	// Drivers Speed
	public static final double kNormalSpeed = 0.5;
	public static final double kNormalTurnSpeed = 0.35;
	public static final double kSlowSpeed = 0.2;
	public static final double kSlowTurnSpeed = 0.25;
		
	// Gyro Constants
	public static final int kGyroZ = 2;
	public static final double kGyroDeadband = 0.1;
	  
	// Motor setup constants
	public static final int kPIDLoopIdx = 0;
	public static final int kTimeoutMs = 30;
	public static final int kBaseTrajPeriodMs = 0;
	public static final double kNeutralDeadband  = 0.01;
    
	public static final double kPickerElevatorKf = 0.034;
	public static final double kPickerElevatorKi = 0.000069;
	public static final double kPickerElevatorKp = 0.0035;
	public static final double kPickerElevatorKd = 0.0;

	// Motor Power Levels
	public static final double climbMotorPowerRaise = 0.9;
	public static final double climbMotorPowerLower= 0.9;
	
	// Timeout durations
	public static final double climbMotorRaiseTimeout = 4.0;
	public static final double climbMotorLowerTimeout = 4.0;
	
	//below are values for navigation
	
	public static final double drivekAngleSetpoint = 0.0;
	public static final double drivekP = 0.004; // propotional turning constant
	public static final double drivekD = 0.001; // propotional turning constant
	public static final double drivekI = 0.001; // propotional turning constant
    public static final double drivekMaxAcum = 1000;
	
    public static final double leftDrivekAngleSetpoint = 0.0;
	public static final double leftDrivekP = 0.004; // propotional turning constant
	public static final double leftDrivekD = 0.001; // propotional turning constant
	public static final double leftDrivekI = 0.001; // propotional turning constant
    public static final double leftDrivekMaxAcum = 1000;
    
    public static final double rightDrivekAngleSetpoint = 0.0;
	public static final double rightDrivekP = 0.004; // propotional turning constant
	public static final double rightDrivekD = 0.001; // propotional turning constant
	public static final double rightDrivekI = 0.001; // propotional turning constant
    public static final double rightDrivekMaxAcum = 1000;
    
    //below are values for PING)) PID control
 // proportional speed constant
 	private static final double pingkP = 7.0;

 	// integral speed constant
 	private static final double pingkI = 0.018;

 	// derivative speed constant
 	private static final double pingkD = 1.5;
 	
 	// max distance in cm
 	private static final double pingMaxD = 100;
 	
 	// hold distance in cm
 	private static final double pingHoldD = 10;
 	
    
	//TODO refactor for 2018
	//below are values from 2017
	//assorted variables
	public static final double collectorSpeed = .5;
	public static final double ellevatorSpeedUp = 1;
	public static final double ellevatorSpeedDown = -.75;
	public static final double shooterSpeed = 1;
	public static final double climber1Speed = 1;
	public static final double climber2Speed = 1;
	public static final double rightSpeed = 1;
	public static final double leftSpeed = .9;
	public static final double curve = 0;//-0.2;

	
	//wheel info
	public static final int wheelDiaIn = 6; //in
	public static final double wheelCircIN = 6*Math.PI; //in
	public static final double wheelCircFT = wheelCircIN/12;
	
	//encoders
	public static final int E4TEncoderPulsePerRev = 1440;
	public static final double encoderRightDistanceFTPerPulse = wheelCircFT/E4TEncoderPulsePerRev;
	public static final double encoderLeftDistanceFTPerPulse = wheelCircFT/E4TEncoderPulsePerRev;
	public static final double encoderConversionDisFT = wheelCircFT/E4TEncoderPulsePerRev; //FT/pulse
	public static final double encoderConversionDisIN = wheelCircIN/E4TEncoderPulsePerRev; //FT/pulse
	
	//Motor IDs
	public static final int rightMotor1SpeedControl = 6; //normally 6
	public static final int rightMotor2SpeedControl = 7;
	public static final int left1MotorSpeedControl = 1; //normally 1
	public static final int left2MotorSpeedControl = 2;
	public static final int climbMotor1SpeedControl = 8; //normally 8
	public static final int climbMotor2SpeedControl = 4;
	public static final int ShooterMotorSpeedControl = 10;
	public static final int ellevatorMotorSpeedControl = 9; //normally 9
	public static final int agitatorMotorSpeedControl = 5;
	public static final int collectorMotorControl = 3;
	
	//servos
	public static final int shooterDoorPort = 1;
	public static final int RBarrelBlockerPort = 2;
	public static final int LBarrelBlockerPort = 3;
	
	//autoconfig
	public final static double kTolerance = 0.1;
	public final static double kP = .88;//-1.0 / 5.0;
	public final static double kPtr = .95; //kP for turn right
	public final static double kPrl = .95; //kP for turn left
	public final static double kI = -1.0 / 5.0;
	public final static double kD = -1.0 / 5.0;
	public final static double defaultAutoSpeed = .75;
	public final static double AutoGearDistS1 = 92; //IN
	public final static double AutoGearAngS2 = 30; //deg
	public final static double AutoGearDistS3 = 60; //IN
	public final static double AutoGearAngRadS2 = 2; //feet of radius
	public final static double AutoDriveFWTimed = 2.25; //sec
	public final static double autoFWSpdRight = 0.5;
	public final static double autoFWSpdLeft = 0.5872;//0.671;

}