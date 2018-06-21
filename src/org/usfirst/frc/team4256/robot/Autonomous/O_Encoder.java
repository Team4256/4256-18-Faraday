package org.usfirst.frc.team4256.robot.Autonomous;

import org.usfirst.frc.team4256.robot.SwerveModule;

import com.cyborgcats.reusable.Autonomous.Odometer;

public class O_Encoder extends Odometer {
	private SwerveModule module;
	private ConsumableDouble x = new ConsumableDouble();
	private ConsumableDouble y = new ConsumableDouble();
	
	public O_Encoder(final SwerveModule module) {this.module = module;}
	
	public void updateX() {x.increment(module.deltaXDistance());}
	public void updateY() {y.increment(module.deltaYDistance());}
	@Override
	public void completeLoopUpdate() {updateX();updateY();}

	@Override
	public void init() {}//unused
}
