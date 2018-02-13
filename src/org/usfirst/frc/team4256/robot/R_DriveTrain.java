package org.usfirst.frc.team4256.robot;

import com.cyborgcats.reusable.R_Gyro;

public class R_DriveTrain {
	private static final double pivotToFrontX = 8.45;//inches, pivot point to front wheel tip, x
	private static final double pivotToFrontY = 10.06;//inches, pivot point to front wheel tip, y
	private static final double pivotToFront = Math.sqrt(pivotToFrontX*pivotToFrontX + pivotToFrontY*pivotToFrontY);
	private static final double pivotToAftX = 8.90;//inches, pivot point to aft wheel tip, x
	private static final double pivotToAftY = 16.94;//inches, pivot point to aft wheel tip, y
	private static final double pivotToAft = Math.sqrt(pivotToAftX*pivotToAftX + pivotToAftY*pivotToAftY);
	private static final double rampDownConstant = .1;
	
	private double moduleD_maxSpeed = 65.0;
	private double moduleD_previousAngle = 0.0;
	private double drivetrain_previousSpin = 0.0;

	private R_Gyro gyro;
	private R_SwerveModule moduleA;
	private R_SwerveModule moduleB;
	private R_SwerveModule moduleC;
	private R_SwerveModule moduleD;
	
	public R_DriveTrain(final R_Gyro gyro, final R_SwerveModule moduleA, final R_SwerveModule moduleB, final R_SwerveModule moduleC, final R_SwerveModule moduleD) {
		this.gyro = gyro;
		this.moduleA = moduleA;
		this.moduleB = moduleB;
		this.moduleC = moduleC;
		this.moduleD = moduleD;
	}
	/**
	 * This function prepares each swerve module individually.
	**/
	public void init() {
		moduleA.init(/*reversed traction*/false);//TODO remove these params once electronics is correct
		moduleB.init(/*reversed traction*/true);
		moduleC.init(/*reversed traction*/true);
		moduleD.init(/*reversed traction*/true);
	}
	
	
	public void holonomicCartesian(final double speedX, final double speedY, final double speedSpin) {
		final double speed = Math.sqrt(speedX*speedX + speedY*speedY);
		
		double moduleAX = speedX + speedSpin*pivotToFrontY/pivotToFront;
		double moduleAY = speedY + speedSpin*pivotToFrontX/pivotToFront;
		double moduleBX = moduleAX;//speedX + spin*pivotToFrontY/pivotToFront;
		double moduleBY = speedY - speedSpin*pivotToFrontX/pivotToFront;
		double moduleCX = speedX - speedSpin*pivotToAftY/pivotToAft;
		double moduleCY = speedY + speedSpin*pivotToAftX/pivotToAft;
		double moduleDX = moduleCX;//speedX - spin*pivotToAftY/pivotToAft;
		double moduleDY = speedY - speedSpin*pivotToAftX/pivotToAft;
		
		boolean bad = speed == 0 && speedSpin == 0;
		moduleA.swivelTo(Math.toDegrees(Math.atan2(moduleAX, moduleAY)), bad);
		moduleB.swivelTo(Math.toDegrees(Math.atan2(moduleBX, moduleBY)), bad);
		moduleC.swivelTo(Math.toDegrees(Math.atan2(moduleCX, moduleCY)), bad);
		moduleD.swivelTo(Math.toDegrees(Math.atan2(moduleDX, moduleDY)), bad);
		
		if (isThere(5)) {
			double speedA = Math.sqrt(moduleAX*moduleAX + moduleAY*moduleAY),
					speedB = Math.sqrt(moduleBX*moduleBX + moduleBY*moduleBY),
					speedC = Math.sqrt(moduleCX*moduleCX + moduleCY*moduleCY),
					speedD = Math.sqrt(moduleDX*moduleDX + moduleDY*moduleDY);
			if (bad) {
				moduleA.set(0);	moduleB.set(0);	moduleC.set(0);	moduleD.set(0);
			}else {
				double max = Math.max(speedA, Math.max(speedB, Math.max(speedC, speedD)));
				if (max > 1) {
					speedA /= max;	speedB /= max;	speedC /= max;	speedD /= max;
				}
				moduleA.set(speedA);	moduleB.set(speedB);	moduleC.set(speedC);	moduleD.set(speedD);
			}
		}
	}
	
	
	private double[] speedsFromModuleD() {
		double rawSpeed = Math.abs(moduleD.tractionSpeed());
		if (rawSpeed > moduleD_maxSpeed) {moduleD_maxSpeed = rawSpeed;}
		rawSpeed /= moduleD_maxSpeed;
		
		double tan = 1.0/Math.tan(Math.toRadians(moduleD_previousAngle));//TODO need validate angle?
		if (Double.isNaN(tan)) {tan = 0.0;}
		
		final double rawX = rawSpeed/Math.sqrt(1.0 + tan*tan);
		final double rawY = rawX*tan;
		final double drivetrainX = rawX + drivetrain_previousSpin*pivotToAftY/pivotToAft;
		final double drivetrainY = rawY + drivetrain_previousSpin*pivotToAftX/pivotToAft;
		
		return new double[] {drivetrainX, drivetrainY};
	}
	
	private double[] computeModuleComponents(final double speedX, final double speedY, final double speedSpin) {
		final double moduleAX = speedX + speedSpin*pivotToFrontY/pivotToFront;
		final double moduleAY = speedY + speedSpin*pivotToFrontX/pivotToFront;
		final double moduleBX = moduleAX;//speedX + spin*pivotToFrontY/pivotToFront;
		final double moduleBY = speedY - speedSpin*pivotToFrontX/pivotToFront;
		final double moduleCX = speedX - speedSpin*pivotToAftY/pivotToAft;
		final double moduleCY = speedY + speedSpin*pivotToAftX/pivotToAft;
		final double moduleDX = moduleCX;//speedX - spin*pivotToAftY/pivotToAft;
		final double moduleDY = speedY - speedSpin*pivotToAftX/pivotToAft;
		
		return new double[] {moduleAX, moduleAY, moduleBX, moduleBY, moduleCX, moduleCY, moduleDX, moduleDY};
	}
	
	private double[] speedsError(final double[] speedsDesired, final double[] speedsActual) {
		final double firstIndexDifference = speedsDesired[0] - speedsActual[0];
		final double secondIndexDifference = speedsDesired[1] - speedsDesired[1];
		return new double[] {firstIndexDifference, secondIndexDifference};
	}
	
	
	public void holonomic(final double direction, final double speed, final double speedSpin) {//TODO could combine holonomics
		//{computing actual speed from encoder value of moduleD}
		double chassis_fieldAngle = gyro.getCurrentAngle();
		double speedY_desired = speed*Math.cos(Math.toRadians(R_SwerveModule.convertToRobot(direction, chassis_fieldAngle)));
		double speedX_desired = speed*Math.sin(Math.toRadians(R_SwerveModule.convertToRobot(direction, chassis_fieldAngle)));
		
		double[] speeds_actual = speedsFromModuleD();
		double[] speeds_error = speedsError(new double[] {speedX_desired, speedY_desired}, speeds_actual);
		
		double speedX_new = speedX_desired;
		if (speeds_error[0] < 0.0) {
			speedX_new = speeds_actual[0] + rampDownConstant*speeds_error[0];
		}
		double speedY_new = speedY_desired;
		if (speeds_error[1] < 0.0) {
			speedY_new = speeds_actual[1] + rampDownConstant*speeds_error[1];
		}
		
		double[] moduleSpeeds_forMotor = computeModuleComponents(speedX_new, speedY_new, speedSpin);
		
		boolean bad = speed == 0 && speedSpin == 0;
		moduleA.swivelTo(Math.toDegrees(Math.atan2(moduleSpeeds_forMotor[0], moduleSpeeds_forMotor[1])), bad);
		moduleB.swivelTo(Math.toDegrees(Math.atan2(moduleSpeeds_forMotor[2], moduleSpeeds_forMotor[3])), bad);
		moduleC.swivelTo(Math.toDegrees(Math.atan2(moduleSpeeds_forMotor[4], moduleSpeeds_forMotor[5])), bad);
		moduleD_previousAngle = Math.toDegrees(Math.atan2(moduleSpeeds_forMotor[6], moduleSpeeds_forMotor[7]));
		moduleD.swivelTo(moduleD_previousAngle, bad);
		
		if (isThere(5)) {
			double speedA = Math.sqrt(moduleSpeeds_forMotor[0]*moduleSpeeds_forMotor[0] + moduleSpeeds_forMotor[1]*moduleSpeeds_forMotor[1]),
					speedB = Math.sqrt(moduleSpeeds_forMotor[2]*moduleSpeeds_forMotor[2] + moduleSpeeds_forMotor[3]*moduleSpeeds_forMotor[3]),
					speedC = Math.sqrt(moduleSpeeds_forMotor[4]*moduleSpeeds_forMotor[4] + moduleSpeeds_forMotor[5]*moduleSpeeds_forMotor[5]),
					speedD = Math.sqrt(moduleSpeeds_forMotor[6]*moduleSpeeds_forMotor[6] + moduleSpeeds_forMotor[7]*moduleSpeeds_forMotor[7]);
			if (bad) {
				moduleA.set(0);	moduleB.set(0);	moduleC.set(0);	moduleD.set(0);
			}else {
				double max = Math.max(speedA, Math.max(speedB, Math.max(speedC, speedD)));
				if (max > 1.0) {
					speedA /= max;	speedB /= max;	speedC /= max;	speedD /= max;
				}
				moduleA.set(speedA);	moduleB.set(speedB);	moduleC.set(speedC);	moduleD.set(speedD);
			}
		}
		
		drivetrain_previousSpin = speedSpin;
	}
	
	
	public boolean isThere(final double threshold) {
		return moduleA.isThere(threshold) && moduleB.isThere(threshold) && moduleC.isThere(threshold) && moduleD.isThere(threshold);
	}
	
	
	public void completeLoopUpdate() {
		moduleA.completeLoopUpdate();
		moduleB.completeLoopUpdate();
		moduleC.completeLoopUpdate();
		moduleD.completeLoopUpdate();
	}
}