package org.usfirst.frc.team4256.robot;

import com.cyborgcats.reusable.R_Gyro;

public class R_DriveTrain {
	private static final double pivotToFrontX = 8.45;//inches, pivot point to front wheel tip, x
	private static final double pivotToFrontY = 10.06;//inches, pivot point to front wheel tip, y
	private static final double pivotToAftX = 8.90;//inches, pivot point to aft wheel tip, x
	private static final double pivotToAftY = 16.94;//inches, pivot point to aft wheel tip, y
	
	//private static final double Front = 25.85;//inches, wheel tip to wheel tip
	//private static final double Radius = Math.sqrt(Side*Side + Front*Front);
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
		moduleA.init(false);//TODO remove these params once electronics is correct
		moduleB.init(false);
		moduleC.init(false);
		moduleD.init(false);
	}
//	/**
//	 * 
//	**/
//	public void align(final double increment) {
//		aligning = true;
//		do {
//			moduleA.align(increment);
//			moduleB.align(increment);
//			moduleC.align(increment);
//			moduleD.align(increment);
//		}while ((moduleA.isAligning() || moduleB.isAligning() || moduleC.isAligning() || moduleD.isAligning()));
//		aligning = false;
//		aligned = true;
//	}
	
	public void holonomic2(final double forward, final double strafe, final double spin) {
		double a = strafe - spin*(Side/Radius),b = strafe + spin*(Side/Radius),c = forward - spin*(Front/Radius),d = forward + spin*(Front/Radius);
		boolean bad = forward*forward + strafe*strafe == 0 && spin == 0;
		moduleA.swivelTo(Math.toDegrees(Math.atan2(b,d)), bad);//TODO may need swivelWith
		moduleB.swivelTo(Math.toDegrees(Math.atan2(b,c)), bad);
		moduleC.swivelTo(Math.toDegrees(Math.atan2(a,d)), bad);
		moduleD.swivelTo(Math.toDegrees(Math.atan2(a,c)), bad);
		
		if (isThere(5)) {
			double speedA = Math.sqrt(b*b + d*d),speedB = Math.sqrt(b*b + c*c),speedC = Math.sqrt(a*a + d*d),speedD = Math.sqrt(a*a + c*c);
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
	
	public void holonomic(final double direction, final double speed, final double spin) {//TODO could combine holonomics
		//TODO accept 2 speeds, one from ZED and one from driver. Use max()
		double chassis_fieldAngle = gyro.getCurrentAngle();
		double speedY = speed*Math.cos(Math.toRadians(R_SwerveModule.convertToRobot(direction, chassis_fieldAngle)));
		double speedX = speed*Math.sin(Math.toRadians(R_SwerveModule.convertToRobot(direction, chassis_fieldAngle)));
		
		double moduleAX = speedX + speed*pivotToFrontY;
		double moduleAY = speedY + speed*pivotToFrontX;
		double moduleBX = speedX + speed*pivotToFrontY;
		double moduleBY = speedY - speed*pivotToFrontX;
		double moduleCX = speedX - speed*pivotToAftY;
		double moduleCY = speedY + speed*pivotToAftX;
		double moduleDX = speedX - speed*pivotToAftY;
		double moduleDY = speedY - speed*pivotToAftX;
		
		boolean bad = speed == 0 && spin == 0;
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
	
	public boolean isThere(final double threshold) {//TODO if the change in my pid error has leveled out, then do..., or if speed has gone below threshold (change here and in SwerveModule)
		return moduleA.isThere(threshold) && moduleB.isThere(threshold) && moduleC.isThere(threshold) && moduleD.isThere(threshold);
	}
}