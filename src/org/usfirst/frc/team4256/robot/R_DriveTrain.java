package org.usfirst.frc.team4256.robot;

import com.cyborgcats.reusable.R_Gyro;
import com.cyborgcats.reusable.V_Compass;

public class R_DriveTrain {
	private static final double pivotToFrontX = 8.45;//inches, pivot point to front wheel tip, x
	private static final double pivotToFrontY = 10.06;//inches, pivot point to front wheel tip, y
	private static final double pivotToFront = Math.sqrt(pivotToFrontX*pivotToFrontX + pivotToFrontY*pivotToFrontY);
	private static final double pivotToAftX = 8.90;//inches, pivot point to aft wheel tip, x
	private static final double pivotToAftY = 16.94;//inches, pivot point to aft wheel tip, y
	private static final double pivotToAft = Math.sqrt(pivotToAftX*pivotToAftX + pivotToAftY*pivotToAftY);
	
	private double moduleD_maxSpeed = 70.0;
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
		moduleA.init(/*reversed traction*/true);//TODO remove these params once electronics is correct
		moduleB.init(/*reversed traction*/false);
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
		
		double tan = 1.0/Math.tan(Math.toRadians(V_Compass.validateAngle(moduleD_previousAngle)));
		if (Double.isNaN(tan)) {tan = 0.0;}
		
		final double rawX = rawSpeed/Math.sqrt(1.0 + tan*tan);
		final double rawY = rawX*tan;
		final double drivetrainX = rawX - Math.abs(drivetrain_previousSpin)*pivotToAftY/pivotToAft;
		final double drivetrainY = rawY - Math.abs(drivetrain_previousSpin)*pivotToAftX/pivotToAft;
		
		return new double[] {drivetrainX, drivetrainY};
	}
	
	private double[] computeModuleComponents(final double speedX, final double speedY, final double speedSpin) {
		final double moduleAX = speedX + speedSpin*pivotToFrontY/pivotToFront,
					 moduleAY = speedY + speedSpin*pivotToFrontX/pivotToFront,
					 moduleBX = moduleAX,//speedX + spin*pivotToFrontY/pivotToFront
					 moduleBY = speedY - speedSpin*pivotToFrontX/pivotToFront,
					 moduleCX = speedX - speedSpin*pivotToAftY/pivotToAft,
					 moduleCY = speedY + speedSpin*pivotToAftX/pivotToAft,
					 moduleDX = moduleCX,//speedX - spin*pivotToAftY/pivotToAft;
					 moduleDY = speedY - speedSpin*pivotToAftX/pivotToAft;
		return new double[] {moduleAX, moduleAY, moduleBX, moduleBY, moduleCX, moduleCY, moduleDX, moduleDY};
	}
	
	private double[] computeModuleAngles(final double[] moduleComponents) {
		final double angleA = Math.toDegrees(Math.atan2(moduleComponents[0], moduleComponents[1])),
					 angleB = Math.toDegrees(Math.atan2(moduleComponents[2], moduleComponents[3])),
					 angleC = Math.toDegrees(Math.atan2(moduleComponents[4], moduleComponents[5])),
					 angleD = Math.toDegrees(Math.atan2(moduleComponents[6], moduleComponents[7]));
		return new double[] {angleA, angleB, angleC, angleD};
	}
	
	private double[] computeModuleSpeeds(final double[] moduleComponents) {
		final double speedA = Math.sqrt(moduleComponents[0]*moduleComponents[0] + moduleComponents[1]*moduleComponents[1]),
					 speedB = Math.sqrt(moduleComponents[2]*moduleComponents[2] + moduleComponents[3]*moduleComponents[3]),
					 speedC = Math.sqrt(moduleComponents[4]*moduleComponents[4] + moduleComponents[5]*moduleComponents[5]),
					 speedD = Math.sqrt(moduleComponents[6]*moduleComponents[6] + moduleComponents[7]*moduleComponents[7]);
		double max = Math.max(speedA, Math.max(speedB, Math.max(speedC, speedD)));
		if (max < 1.0) {max = 1.0;}
		return new double[] {speedA/max, speedB/max, speedC/max, speedD/max};
	}
	
//	private double[] speedsError(final double[] speedsDesired, final double[] speedsActual) {
//		final double firstIndexDifference = speedsDesired[0] - speedsActual[0];
//		final double secondIndexDifference = speedsDesired[1] - speedsDesired[1];
//		return new double[] {firstIndexDifference, secondIndexDifference};
//	}
	
	
	public void holonomic(final double direction, final double speed, final double speedSpin) {//TODO could combine holonomics
		//{computing actual speed from encoder value of moduleD}
		final double chassis_fieldAngle = gyro.getCurrentAngle();
		final double speedY_desired = speed*Math.cos(Math.toRadians(R_SwerveModule.convertToRobot(direction, chassis_fieldAngle))),
					 speedX_desired = speed*Math.sin(Math.toRadians(R_SwerveModule.convertToRobot(direction, chassis_fieldAngle)));
		final double[] moduleComps_desired = computeModuleComponents(speedX_desired, speedY_desired, speedSpin);
		
		final double[] speeds_actual = speedsFromModuleD();
		double speed_actual = Math.sqrt(speeds_actual[0]*speeds_actual[0] + speeds_actual[1]*speeds_actual[1]);
		speed_actual = Math.floor(speed_actual*10.0)/10.0;
		
		final double[] moduleAngles_final;
		if ((speed < speed_actual) && (speed_actual > .2)) {
			final double[] moduleAngles_desired = computeModuleAngles(moduleComps_desired);
			final double max_desired = Math.max(moduleAngles_desired[0], Math.max(moduleAngles_desired[1], Math.max(moduleAngles_desired[2], moduleAngles_desired[3]))),
						 min_desired = Math.min(moduleAngles_desired[0], Math.max(moduleAngles_desired[1], Math.max(moduleAngles_desired[2], moduleAngles_desired[3])));
			final double range_desired = max_desired - min_desired;
			
			final double[] moduleComps_actual = computeModuleComponents(speeds_actual[0], speeds_actual[1], speedSpin);
			final double[] moduleAngles_actual = computeModuleAngles(moduleComps_actual);
			final double max_actual = Math.max(moduleAngles_actual[0], Math.max(moduleAngles_actual[1], Math.max(moduleAngles_actual[2], moduleAngles_actual[3]))),
						 min_actual = Math.min(moduleAngles_actual[0], Math.max(moduleAngles_actual[1], Math.max(moduleAngles_actual[2], moduleAngles_actual[3])));
			final double range_actual = max_actual - min_actual;
			
			moduleAngles_final = range_desired > range_actual ? moduleAngles_actual : moduleAngles_desired;
		}else {
			moduleAngles_final = computeModuleAngles(moduleComps_desired);
		}
		
		boolean bad = speed == 0.0 && speedSpin == 0.0;
		moduleA.swivelTo(moduleAngles_final[0], bad);	moduleB.swivelTo(moduleAngles_final[1], bad);
		moduleC.swivelTo(moduleAngles_final[2], bad);	moduleD.swivelTo(moduleAngles_final[3], bad);
		moduleD_previousAngle = moduleAngles_final[3];
		
		if (isThere(5)) {
			final double[] moduleSpeeds_final = computeModuleSpeeds(moduleComps_desired);
			if (bad) {
				moduleA.set(0.0);						moduleB.set(0.0);
				moduleC.set(0.0);						moduleD.set(0.0);
			}else {
				moduleA.set(moduleSpeeds_final[0]);		moduleB.set(moduleSpeeds_final[1]);
				moduleC.set(moduleSpeeds_final[2]);		moduleD.set(moduleSpeeds_final[3]);
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