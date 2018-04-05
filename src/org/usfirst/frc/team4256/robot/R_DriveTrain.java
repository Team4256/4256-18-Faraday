package org.usfirst.frc.team4256.robot;

import com.cyborgcats.reusable.R_Gyro;
import com.cyborgcats.reusable.V_Compass;

public class R_Drivetrain {
	private static final double pivotToFrontX = 8.45;//inches, pivot point to front wheel tip, x
	private static final double pivotToFrontY = 10.06;//inches, pivot point to front wheel tip, y
	private static final double pivotToFront = Math.hypot(pivotToFrontX, pivotToFrontY);
	private static final double pivotToAftX = 8.90;//inches, pivot point to aft wheel tip, x
	private static final double pivotToAftY = 16.94;//inches, pivot point to aft wheel tip, y
	private static final double pivotToAft = Math.hypot(pivotToAftX, pivotToAftY*pivotToAftY);
	
	private double moduleD_maxSpeed = 140.0;//always put max slightly higher than max observed
	private double moduleD_previousAngle = 0.0;
	private double previousSpin = 0.0;

	public final R_Gyro gyro;
	private final R_SwerveModule moduleA, moduleB, moduleC, moduleD;
	private final R_SwerveModule[] modules;
	
	
	public R_Drivetrain(final R_Gyro gyro, final R_SwerveModule moduleA, final R_SwerveModule moduleB, final R_SwerveModule moduleC, final R_SwerveModule moduleD) {
		this.gyro = gyro;
		this.moduleA = moduleA;
		this.moduleB = moduleB;
		this.moduleC = moduleC;
		this.moduleD = moduleD;
		this.modules = new R_SwerveModule[] {moduleA, moduleB, moduleC, moduleD};
	}
	
	/**
	 * This function prepares each swerve module individually.
	**/
	public void init() {
		moduleA.init(/*reversed traction*/true);//practice: true, comp: false
		moduleB.init(/*reversed traction*/true);//practice: true, comp: false
		moduleC.init(/*reversed traction*/false);//practice: false, comp: false
		moduleD.init(/*reversed traction*/true);//practice: true, comp: true
	}
	
	
	public void holonomic(final double direction, double speed, final double spin) {
		//{PREPARE VARIABLES}
		speed = Math.abs(speed);
		final double chassis_fieldAngle = gyro.getCurrentAngle();
		final double forward = speed*Math.cos(Math.toRadians(R_SwerveModule.convertToRobot(direction, chassis_fieldAngle))),
					 strafe  = speed*Math.sin(Math.toRadians(R_SwerveModule.convertToRobot(direction, chassis_fieldAngle)));
		final double[] comps_desired = computeComponents(strafe, forward, spin);
		final boolean bad = speed == 0.0 && spin == 0.0;
		
		//{GET ENCODER SPEED}
		final double[] speeds_actual = speedsFromModuleD();
		double speed_actual = Math.hypot(speeds_actual[0], speeds_actual[1]);
		speed_actual = Math.floor(speed_actual*10.0)/10.0;
		
		//{COMPUTE ANGLES}
		final double[] angles_final;
		if ((speed < speed_actual) && (speed_actual > .1)) {
			final double[] angles_desired = computeAngles(comps_desired);
			final double stdd_desired = V_Compass.stdd(angles_desired);
			
			final double[] angles_actual = computeAngles(computeComponents(speeds_actual[0], speeds_actual[1], spin));
			final double stdd_actual = V_Compass.stdd(angles_actual);
			
			angles_final = stdd_desired > stdd_actual ? angles_actual : angles_desired;
		}else {
			angles_final = computeAngles(comps_desired);
		}
		
		//{CONTROL MOTORS, using above angles and computing traction outputs as needed}
		if (!bad) {
			for (int i = 0; i < 4; i++) modules[i].swivelTo(angles_final[i]);//control rotation if good
			moduleD_previousAngle = angles_final[3];
		}
		
		if (!bad && isThere(6.0)) {
			final double[] speeds_final = computeSpeeds(comps_desired);
			for (int i = 0; i < 4; i++) modules[i].set(speeds_final[i]);//control traction if good and there
		}else stop();//otherwise, stop traction
		
		//{UPDATE RECORDS}
		previousSpin = spin;
	}
	
	
	public void holonomic_encoderIgnorant(final double direction, double speed, final double speedSpin) {
		//{PREPARE VARIABLES}
		speed = Math.abs(speed);
		final double chassis_fieldAngle = gyro.getCurrentAngle();
		final double forward = speed*Math.cos(Math.toRadians(R_SwerveModule.convertToRobot(direction, chassis_fieldAngle))),
					 strafe  = speed*Math.sin(Math.toRadians(R_SwerveModule.convertToRobot(direction, chassis_fieldAngle)));
		final double[] comps_desired = computeComponents(strafe, forward, speedSpin);
		final boolean bad = speed == 0.0 && speedSpin == 0.0;
		
		//{CONTROL MOTORS, computing outputs as needed}
		if (!bad) {
			final double[] angles_final = computeAngles(comps_desired);
			for (int i = 0; i < 4; i++) modules[i].swivelTo(angles_final[i]);//control rotation if good
		}
		
		if (!bad && isThere(6.0)) {
			final double[] speeds_final = computeSpeeds(comps_desired);
			for (int i = 0; i < 4; i++) modules[i].set(speeds_final[i]);//control traction if good and there
		}else stop();//otherwise, stop traction
	}
	
	
	private double[] speedsFromModuleD() {
		double rawSpeed = moduleD.tractionSpeed()*moduleD.decapitated();
		if (Math.abs(rawSpeed) > moduleD_maxSpeed) moduleD_maxSpeed = Math.abs(rawSpeed);
		rawSpeed /= moduleD_maxSpeed;
		
		final double angle = Math.toRadians(moduleD_previousAngle);
		
		final double drivetrainX = /*linear*/rawSpeed*Math.sin(angle) + /*rotational*/previousSpin*pivotToAftY/pivotToAft;
		final double drivetrainY = /*linear*/rawSpeed*Math.cos(angle) + /*rotational*/previousSpin*pivotToAftX/pivotToAft;
		
		return new double[] {drivetrainX, drivetrainY};
	}
	
	
	public void formX() {moduleA.swivelTo(-45.0); moduleB.swivelTo(45.0); moduleC.swivelTo(45.0); moduleD.swivelTo(-45.0);}
	public boolean isThere(final double threshold) {
		return moduleA.isThere(threshold) && moduleB.isThere(threshold) && moduleC.isThere(threshold) && moduleD.isThere(threshold);
	}
	public void autoMode(final boolean enable) {for (R_SwerveModule module : modules) module.autoMode(enable);}
	public void stop() {for (R_SwerveModule module : modules) module.set(0.0);}
	public void completeLoopUpdate() {for (R_SwerveModule module : modules) module.completeLoopUpdate();}
	
	
	
	//-------------------------------------------------COMPUTATION CODE------------------------------------------
	private static double[] computeComponents(final double speedX, final double speedY, final double speedSpin) {
		return new double[] {
			speedX + speedSpin*pivotToFrontY/pivotToFront,//moduleAX
			speedY + speedSpin*pivotToFrontX/pivotToFront,//moduleAY
			speedX + speedSpin*pivotToFrontY/pivotToFront,//moduleBX
			speedY - speedSpin*pivotToFrontX/pivotToFront,//moduleBY
			speedX - speedSpin*pivotToAftY/pivotToAft,//moduleCX
			speedY + speedSpin*pivotToAftX/pivotToAft,//moduleCY
			speedX - speedSpin*pivotToAftY/pivotToAft,//moduleDX
			speedY - speedSpin*pivotToAftX/pivotToAft//moduleDY
		};
	}
	
	
	private static double[] computeAngles(final double[] moduleComponents) {
		double[] angles = new double[4];
		for (int i = 0; i < 4; i++) angles[i] = Math.toDegrees(Math.atan2(moduleComponents[i*2], moduleComponents[i*2 + 1]));
		return angles;
	}
	
	
	private static double[] computeSpeeds(final double[] moduleComponents) {
		//don't use for loop because of max divide
		final double speedA = Math.hypot(moduleComponents[0], moduleComponents[1]),
					 speedB = Math.hypot(moduleComponents[2], moduleComponents[3]),
					 speedC = Math.hypot(moduleComponents[4], moduleComponents[5]),
					 speedD = Math.hypot(moduleComponents[6], moduleComponents[7]);
		double max = Math.max(speedA, Math.max(speedB, Math.max(speedC, speedD)));
		if (max < 1.0) {max = 1.0;}
		return new double[] {speedA/max, speedB/max, speedC/max, speedD/max};
	}
}