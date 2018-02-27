//DRIVER
//Back: align gyro
//left stick, both axis: raw speed and direction
//right stick, x axis: raw spin
//left stick, press: turbo mode
//right stick, press: snail mode
//LB: slurp (intake in)
//LT: increment elevator down
//RB: spit (intake out)
//RT: increment elevator up
//B: elevator low scale preset
//A: elevator floor preset
//X: elevator switch preset
//Y: elevator high scale preset

package org.usfirst.frc.team4256.robot;

public abstract class Parameters {
	//ELECTRONICS
	public static final int Swerve_rotatorA = 11;//CAN, Talon SRX, front left, PDP 4
	public static final int Swerve_rotatorB = 12;//CAN, Talon SRX, front right, PDP 5
	public static final int Swerve_rotatorC = 13;//CAN, Talon SRX, aft left, PDP 6
	public static final int Swerve_rotatorD = 14;//CAN, Talon SRX, aft right, PDP 7
	public static final int Swerve_driveA = 21;//CAN, Talon SRX, front left, PDP 0
	public static final int Swerve_driveB = 22;//CAN, Talon SRX, front right, PDP 1
	public static final int Swerve_driveC = 23;//CAN, Talon SRX, aft left, PDP 2
	public static final int Swerve_driveD = 24;//CAN, Talon SRX, aft right, PDP 3
	
	public static final int Intake_left = 15;//CAN, Victor SPX, PDP 8
	public static final int Intake_right = 16;//CAN, Victor SPX, PDP 9
	
	public static final int ElevatorOne_master = 26;//CAN, Talon SRX, Master, PDP 12, First Stage
	public static final int ElevatorOne_followerA = 27;//CAN, Victor SPX, Follower, PDP 13, First Stage
	public static final int ElevatorOne_followerB = 28;//CAN, Victor SPX, Follower, PDP 14, First Stage
	
	public static final int ElevatorTwo_master = 17;//CAN, Talon SRX, PDP 10, Second Stage
	
	public static final int clampyRotator = 29;//CAN, Talon SRX, PDP 15
	
	public static final int ultrasonic = 1;//AIO
	
	//PNEUMATICS
	public static final int ElevatorOne_shifterModule = 0;//PCM
	public static final int ElevatorOne_shifterForward = 1;//PCM
	public static final int ElevatorOne_shifterReverse = 2;//PCM
	
	public static final int Clamp_module = 0;//PCM
	public static final int Clamp_forward = 3;//PCM
	public static final int Clamp_reverse = 4;//PCM
	
	public static final int Extender_module = 0;//PCM
	public static final int Extender_forward = 5;//PCM
	public static final int Extender_reverse = 6;//PCM
	
	//AUTONOMOUS
//	public static final double[][] leftInstructions = new double[][] {
//		//duration ms, direction, speed, orientation
//		{2150, -20, .2, Parameters.leftGear},
//		{700, 0, 0, Parameters.leftGear},
//		{250, Parameters.leftGear, .15, Parameters.leftGear}
//	};
//	public static final double[][] middleInstructions = new double[][] {
//		//duration ms, direction, speed, orientation
//		{1000, 0, .15, 0}
//	};
//	public static final double[][] rightInstructions = new double[][] {
//		//duration ms, direction, speed, orientation
//		{2000, 20, .2, Parameters.rightGear},
//		{500, 0.01, 0, Parameters.rightGear},
//		{300, Parameters.rightGear, .15, Parameters.rightGear}
//	};
	
	//VALUES
	public static enum ElevatorPresets {//TODO get accurate numbers
		FLOOR(0),
		SWITCH(30),
		SCALE_LOW(60),
		SCALE_HIGH(80);
		
		private final int height;
		
		ElevatorPresets(final int height) {
			this.height = height;
		}
		
		public int height() {
			return height;
		}
	}
	
	public static final byte Gyrometer_updateHz = 50;
	
	public static final double spinP = .0025;
	public static final double spinI = .000015;
	public static final double spinD = .02;
	
	public static final double zedP = 0.0919;
	public static final double zedI = 0.00002;
	public static final double zedD = 1.0;
}
