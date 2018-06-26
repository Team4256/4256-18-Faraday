package org.usfirst.frc.team4256.robot;

import com.cyborgcats.reusable.Compass;
import com.cyborgcats.reusable.Drivetrain;
import com.cyborgcats.reusable.PID;

public class D_Swerve implements Drivetrain {
	private static final double pivotToFrontX = 8.45,//inches, pivot point to front wheel tip, x
								pivotToFrontY = 10.06,//inches, pivot point to front wheel tip, y
								pivotToAftX = 8.90,//inches, pivot point to aft wheel tip, x
								pivotToAftY = 16.94;//inches, pivot point to aft wheel tip, y
	private static final double pivotToFront = Math.hypot(pivotToFrontX, pivotToFrontY),
								pivotToAft = Math.hypot(pivotToAftX, pivotToAftY);
	
	private final SwerveModule moduleA, moduleB, moduleC, moduleD;
	private final SwerveModule[] modules;
	
	private double moduleD_maxSpeed = 70.0;//always put max slightly higher than max observed
	private double moduleD_previousAngle = 0.0;
	private double previousSpin = 0.0;
	
	private double direction = 0.0, speed = 0.0, spin = 0.0;
	
	
	public D_Swerve(final SwerveModule moduleA, final SwerveModule moduleB, final SwerveModule moduleC, final SwerveModule moduleD) {
		this.moduleA = moduleA;	this.moduleB = moduleB;	this.moduleC = moduleC;	this.moduleD = moduleD;
		this.modules = new SwerveModule[] {moduleA, moduleB, moduleC, moduleD};
	}
	
	/**
	 * This function prepares each swerve module individually.
	**/
	@Override
	public void init() {
		moduleA.init(/*reversed traction*/true);	moduleB.init(/*reversed traction*/false);
		moduleC.init(/*reversed traction*/false);	moduleD.init(/*reversed traction*/true);
	}
	
	
	private void holonomic(final double direction, double speed, final double spin) {
		//{PREPARE VARIABLES}
		speed = Math.abs(speed);
		final double chassis_fieldAngle = Robot.gyroHeading;
		final double forward = speed*Math.cos(Math.toRadians(SwerveModule.convertToRobot(direction, chassis_fieldAngle))),
					 strafe  = speed*Math.sin(Math.toRadians(SwerveModule.convertToRobot(direction, chassis_fieldAngle)));
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
			final double stdd_desired = Compass.stdd(angles_desired);
			
			final double[] angles_actual = computeAngles(computeComponents(speeds_actual[0], speeds_actual[1], spin));
			final double stdd_actual = Compass.stdd(angles_actual);
			
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
		
		if (spin < 0.07) moduleD.checkTractionEncoder();
		
		//{UPDATE RECORDS}
		previousSpin = spin;
	}
	
	
	private void holonomic_encoderIgnorant(final double direction, double speed, final double spin) {
		//{PREPARE VARIABLES}
		speed = Math.abs(speed);
		final double chassis_fieldAngle = Robot.gyroHeading;
		final double forward = speed*Math.cos(Math.toRadians(SwerveModule.convertToRobot(direction, chassis_fieldAngle))),
					 strafe  = speed*Math.sin(Math.toRadians(SwerveModule.convertToRobot(direction, chassis_fieldAngle)));
		final double[] comps_desired = computeComponents(strafe, forward, spin);
		final boolean bad = speed == 0.0 && spin == 0.0;
		
		//{CONTROL MOTORS, computing outputs as needed}
		if (!bad) {
			final double[] angles_final = computeAngles(comps_desired);
			for (int i = 0; i < 4; i++) modules[i].swivelTo(angles_final[i]);//control rotation if good
		}
		
		if (!bad && isThere(10.0)) {
			final double[] speeds_final = computeSpeeds(comps_desired);
			for (int i = 0; i < 4; i++) modules[i].set(speeds_final[i]);//control traction if good and there
		}else stop();//otherwise, stop traction
		
		if (spin < 0.07) moduleD.checkTractionEncoder();
	}
	
	
	public boolean align() {
		final boolean a = moduleA.magneticAlignment(-90.0),
					  b = moduleB.magneticAlignment(180.0),
					  c = moduleC.magneticAlignment(0.0),
					  d = moduleD.magneticAlignment(90.0);
		return a || b || c || d;
	}
	
	private double[] speedsFromModuleD() {
		double rawSpeed = moduleD.tractionSpeed()*moduleD.decapitated();
		if (Math.abs(rawSpeed) > moduleD_maxSpeed) moduleD_maxSpeed = Math.abs(rawSpeed);
		rawSpeed /= moduleD_maxSpeed;
		
		final double angle = Math.toRadians(moduleD_previousAngle);
		
		final double drivetrainX = /*linear*/rawSpeed*Math.sin(angle) + /*rotational*/previousSpin*pivotToAftY/pivotToAft*Math.signum(rawSpeed);
		final double drivetrainY = /*linear*/rawSpeed*Math.cos(angle) + /*rotational*/previousSpin*pivotToAftX/pivotToAft*Math.signum(rawSpeed);
		
		return new double[] {drivetrainX, drivetrainY};
	}
	
	public void formX() {moduleA.swivelTo(-45.0); moduleB.swivelTo(45.0); moduleC.swivelTo(45.0); moduleD.swivelTo(-45.0);}
	public boolean isThere(final double threshold) {
		return moduleA.isThere(threshold) && moduleB.isThere(threshold) && moduleC.isThere(threshold) && moduleD.isThere(threshold);
	}
	public void autoMode(final boolean enable) {for (SwerveModule module : modules) module.autoMode(enable);}
	private void stop() {for (SwerveModule module : modules) module.set(0.0);}
	@Override
	public void completeLoopUpdate() {
		holonomic_encoderIgnorant(direction, speed, spin);
		for (SwerveModule module : modules) module.completeLoopUpdate();
	}
	
	
	
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

	
	//------------------------------------------------CONFORMING CODE----------------------------------------
	@Override
	public void setSpeed(final double speed) {this.speed = speed <= 1.0 ? speed : 1.0;}
	@Override
	public void setSpin(final double speed) {this.spin = Math.abs(speed) <= 1.0 ? speed : Math.signum(speed);}
	@Override
	public void travelTowards(final double heading) {this.direction = heading;}

	@Override
	public void correctFor(final double errorDirection, final double errorMagnitude) {
		travelTowards(errorDirection);
		
		double speed = PID.get("zed", errorMagnitude);//DO NOT use I gain with this because errorMagnitude is always positive
		if (speed > 0.6) speed = 0.6;
		
		setSpeed(speed);
	}
	
	@Override
	public double face(final double heading, double maximumOutput) {
		final double error = Compass.path(Robot.gyroHeading, heading);
		final double spin = PID.get("spin", error);
		setSpin(Math.max(-maximumOutput, Math.min(spin, maximumOutput)));
		return error;
	}
}