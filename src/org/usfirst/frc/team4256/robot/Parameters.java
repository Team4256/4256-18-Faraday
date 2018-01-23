package org.usfirst.frc.team4256.robot;

public abstract class Parameters {
	//ELECTRONICS
	public static final int ClimberA = 18;//CAN, master
	public static final int ClimberB = 19;//CAN, slave
	
	public static final int Camera_servoX = 8;//PWM
	public static final int Camera_servoY = 9;//PWM
	
	public static final int Lift = 15;//CAN
	
	public static final int Shooter_flywheel = 16;//CAN
	public static final int Shooter_rotator = 17;//CAN
	public static final int Shooter_linearServos = 5;//PWM
	public static final int Shooter_calibrator = 6;//DIO
	
	public static final int Swerve_rotatorA = 11;//CAN, front left
	public static final int Swerve_rotatorB = 12;//CAN, front right
	public static final int Swerve_rotatorC = 13;//CAN, aft left
	public static final int Swerve_rotatorD = 14;//CAN, aft right
	public static final int Swerve_driveA = 21;//CAN, front left
	public static final int Swerve_driveB = 22;//CAN, front right
	public static final int Swerve_driveC = 23;//CAN, aft left
	public static final int Swerve_driveD = 24;//CAN, aft right
	
	//PNEUMATICS
	public static final int Shooter_flapModule = 0;//PCM
	public static final int Shooter_flapForward = 2;//PCM
	public static final int Shooter_flapReverse = 3;//PCM
	
	public static final int Gearer_module = 0;//PCM
	public static final int Gearer_forward = 0;//PCM
	public static final int Gearer_reverse = 1;//PCM
	
	public static final int Clamp_module = 0;//PCM
	public static final int Clamp_forward = 2;//PCM
	public static final int Clamp_reverse = 3;//PCM
	
	//AUTONOMOUS
	public static final double[][] leftInstructions = new double[][] {
		//duration ms, direction, speed, orientation
		{2150, -20, .2, Parameters.leftGear},
		{700, 0, 0, Parameters.leftGear},
		{250, Parameters.leftGear, .15, Parameters.leftGear}
	};
	public static final double[][] middleInstructions = new double[][] {
		//duration ms, direction, speed, orientation
		{1000, 0, .15, 0}
	};
	public static final double[][] rightInstructions = new double[][] {
		//duration ms, direction, speed, orientation
		{2000, 20, .2, Parameters.rightGear},
		{500, 0.01, 0, Parameters.rightGear},
		{300, Parameters.rightGear, .15, Parameters.rightGear}
	};
	
	//VALUES
	public static final byte Gyrometer_updateHz = 50;
	
	public static final double leftGear = 60;
	public static final double centerGear = 0;
	public static final double rightGear = 330;
	public static double loadingStation = 320;
	
	public static final double swerveP = .6;
	public static final double swerveI = 0;
	public static final double swerveD = 6;
	
	public static final double spinP = .0025;
	public static final double spinI = .000015;
	public static final double spinD = .02;
	
	public static final double forwardP = .0025;//TODO tune with TK1
	public static final double forwardI = 0;
	public static final double forwardD = 0;
	
	public static final double strafeP = .0025;//TODO tune with TK1
	public static final double strafeI = 0;
	public static final double strafeD = 0;
}
