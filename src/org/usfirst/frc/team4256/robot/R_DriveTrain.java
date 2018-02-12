package org.usfirst.frc.team4256.robot;

import com.cyborgcats.reusable.R_Gyro;

public class R_DriveTrain {
	private static final double pivotToFrontX = 8.45;//inches, pivot point to front wheel tip, x
	private static final double pivotToFrontY = 10.06;//inches, pivot point to front wheel tip, y
	private static final double pivotToFront = Math.sqrt(pivotToFrontX*pivotToFrontX + pivotToFrontY*pivotToFrontY);
	private static final double pivotToAftX = 8.90;//inches, pivot point to aft wheel tip, x
	private static final double pivotToAftY = 16.94;//inches, pivot point to aft wheel tip, y
	private static final double pivotToAft = Math.sqrt(pivotToAftX*pivotToAftX + pivotToAftY*pivotToAftY);
	
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
		final double rawSpeed = Math.abs(moduleD.tractionSpeed())/65.0;//65.0 is the empirically maximum speed
		
		final double tan = 1/Math.tan(Math.toRadians(moduleD_previousAngle));//TODO need validate angle?
		final double rawX = rawSpeed/Math.sqrt(1 + tan*tan);
		final double rawY = rawX*tan;
		final double drivetrainX = rawX + drivetrain_previousSpin*pivotToAftY/pivotToAft;
		final double drivetrainY = rawY + drivetrain_previousSpin*pivotToAftX/pivotToAft;
		
		return new double[] {drivetrainX, drivetrainY};
	}
	
	
	public void holonomic(final double direction, final double speed, final double spin) {//TODO could combine holonomics
		//{computing actual speed from encoder value of moduleD}
		double chassis_fieldAngle = gyro.getCurrentAngle();
		double speedY_desired = speed*Math.cos(Math.toRadians(R_SwerveModule.convertToRobot(direction, chassis_fieldAngle)));
		double speedX_desired = speed*Math.sin(Math.toRadians(R_SwerveModule.convertToRobot(direction, chassis_fieldAngle)));
		
		double[] actualSpeeds = speedsFromModuleD();
		double speedX = Math.max(speedX_desired, actualSpeeds[0]);
		double speedY = Math.max(speedY_desired, actualSpeeds[1]);
		
		double moduleAX = speedX + spin*pivotToFrontY/pivotToFront;
		double moduleAY = speedY + spin*pivotToFrontX/pivotToFront;
		double moduleBX = moduleAX;//speedX + spin*pivotToFrontY/pivotToFront;
		double moduleBY = speedY - spin*pivotToFrontX/pivotToFront;
		double moduleCX = speedX - spin*pivotToAftY/pivotToAft;
		double moduleCY = speedY + spin*pivotToAftX/pivotToAft;
		double moduleDX = moduleCX;//speedX - spin*pivotToAftY/pivotToAft;
		double moduleDY = speedY - spin*pivotToAftX/pivotToAft;
		
		moduleD_previousAngle = Math.toDegrees(Math.atan2(moduleDX, moduleDY));
		
		boolean bad = speed == 0 && spin == 0;
		moduleA.swivelTo(Math.toDegrees(Math.atan2(moduleAX, moduleAY)), bad);
		moduleB.swivelTo(Math.toDegrees(Math.atan2(moduleBX, moduleBY)), bad);
		moduleC.swivelTo(Math.toDegrees(Math.atan2(moduleCX, moduleCY)), bad);
		moduleD.swivelTo(moduleD_previousAngle, bad);
		
		if (isThere(5)) {
			double speedA = Math.sqrt(moduleAX*moduleAX + moduleAY*moduleAY),
					speedB = Math.sqrt(moduleBX*moduleBX + moduleBY*moduleBY),
					speedC = Math.sqrt(moduleCX*moduleCX + moduleCY*moduleCY),
					speedD = Math.sqrt(moduleDX*moduleDX + moduleDY*moduleDY);
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
		
		drivetrain_previousSpin = spin;
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