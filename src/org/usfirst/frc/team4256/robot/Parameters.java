//DRIVER
//Back: align gyro
//left stick, both axis: raw speed and direction
//right stick, x axis: raw spin
//left stick, press: turbo mode
//right stick, press: snail mode
//RT: slurp (intake in)
//LT: spit
//RB: manually close clamp
//LB: manually open clamp

//GUNNER
//RT: elevator up
//LT: elevator down
//RB: increment elevator up
//LB: increment elevator down
//B: elevator low scale preset
//A: elevator floor preset
//X: elevator switch preset
//Y: elevator high scale preset

package org.usfirst.frc.team4256.robot;

public final class Parameters {
	private Parameters() {}
	
	//ELECTRONICS
	public static final int
	rotationAID = 11,//CAN, Talon SRX, front left, PDP 4
	rotationBID = 12,//CAN, Talon SRX, front right, PDP 5
	rotationCID = 13,//CAN, Talon SRX, aft left, PDP 6
	rotationDID = 14,//CAN, Talon SRX, aft right, PDP 7
	tractionAID = 21,//CAN, Talon SRX, front left, PDP 0
	tractionBID = 22,//CAN, Talon SRX, front right, PDP 1
	tractionCID = 23,//CAN, Talon SRX, aft left, PDP 2
	tractionDID = 24,//CAN, Talon SRX, aft right, PDP 3
	magnetAID = 0,//Digital Input ID for Magnetic Alignment
	magnetBID = 1,//Digital Input ID for Magnetic Alignment
	magnetCID = 2,//Digital Input ID for Magnetic Alignment
	magnetDID = 3;//Digital Input ID for Magnetic Alignment
	
	public static final int INTAKE_LEFT = 15;//CAN, Victor SPX, PDP 8
	public static final int INTAKE_RIGHT = 16;//CAN, Victor SPX, PDP 9
	
	public static final int ELEVATOR_ONE_MASTER = 26;//CAN, Talon SRX, Master, PDP 12, First Stage
	public static final int ELEVATOR_ONE_FOLLOWER_A = 28;//CAN, Victor SPX, Follower, PDP 13, First Stage
	public static final int ELEVATOR_ONE_FOLLOWER_B = 27;//CAN, Victor SPX, Follower, PDP 14, First Stage
	
	public static final int ELEVATOR_TWO_MASTER = 17;//CAN, Talon SRX, PDP 10, Second Stage
	
	public static final int CLAMPY_ROTATOR = 29;//CAN, Talon SRX, PDP 15
	
	public static final int PRESSURE_GAUGE = 0;//AIO
	public static final int ULTRASONIC = 1;//AIO
	public static final int TX2_POWER_SENSOR = 8;//DIO
	public static final int TX2_POWER_CONTROL = 9;//DIO
	
	//PNEUMATICS
	public static final int ELEVATOR_ONE_SHIFTER_MODULE = 0;//PCM
	public static final int ELEVATOR_ONE_SHIFTER_FORWARD = 1;//PCM
	public static final int ELEVATOR_ONE_SHIFTER_REVERSE = 2;//PCM
	
	public static final int CLAMP_MODUlE = 0;//PCM
	public static final int CLAMP_FORWARD = 3;//PCM
	public static final int CLAMP_REVERSE = 4;//PCM
	
	public static final int EXTENDER_MODULE = 0;//PCM
	public static final int EXTENDER_FORWARD = 5;//PCM
	public static final int EXTENDER_REVERSE = 6;//PCM
	
	
	//VALUES
	public static enum ElevatorPresets {
		FLOOR(0),
		SWITCH(30),
		SCALE_LOW(60),
		SCALE_HIGH(80);
		
		private final int height;
		
		ElevatorPresets(final int height) {this.height = height;}
		
		public int height() {return height;}
		public String heightString() {return Double.toString(height);}
	}
	
	public static final byte Gyrometer_updateHz = 50;
	
	public static final double SPIN_P = 0.05;
	public static final double SPIN_I = 0.000015;
	public static final double SPIN_D = 4.25;
	
	//.1838 works well for a leash length of 3, doubling that works for length of 1.5
	public static final double LEASH_P = 0.1838*2.0;
	public static final double LEASH_I = 0.0;
	public static final double LEASH_D = 1.2;
}
