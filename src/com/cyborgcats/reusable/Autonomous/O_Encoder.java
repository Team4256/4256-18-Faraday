package com.cyborgcats.reusable.Autonomous;

import org.usfirst.frc.team4256.robot.R_SwerveModule;

public class O_Encoder extends Odometer {
	private R_SwerveModule module;
	private double tareX = 0.0, tareY = 0.0;
	private ConsumableDouble x = new ConsumableDouble();
	private ConsumableDouble y = new ConsumableDouble();
	
	public O_Encoder(final R_SwerveModule module) {
		this.module = module;
	}
	
	public void updateX() {x.increment(module.deltaXDistance());}
	public void updateY() {y.increment(module.deltaYDistance());}
	public void update() {updateX();updateY();}

	@Override
	public void init() {}//unused

	@Override
	public void setOrigin(final double x, final double y) {tareX = x;	tareY = y;}

	@Override
	public double getX() {return x.get() - tareX;}

	@Override
	public double getY() {return y.get() - tareY;}

	@Override
	public boolean newX() {return x.isNew();}

	@Override
	public boolean newY() {return y.isNew();}

	@Override
	public void disable() {}//unused
}
