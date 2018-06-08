package com.cyborgcats.reusable.Autonomous;

public abstract class Odometer {
	public abstract void init();
	public abstract void setOrigin(final double x, final double y);
	
	public abstract double getX();
	public abstract double getY();
	public abstract boolean newX();
	public abstract boolean newY();
	public abstract void update();
	public abstract void disable();
	
	protected class ConsumableDouble {
		private boolean isNew = false;
		private double value = 0.0;
		
		public void set(final double val) {this.value = val;	isNew = true;}
		public void increment(final double val) {this.value += val;		isNew = true;}
		public double get() {isNew = false;		return value;}
		public boolean isNew() {return isNew;}
	}
}
