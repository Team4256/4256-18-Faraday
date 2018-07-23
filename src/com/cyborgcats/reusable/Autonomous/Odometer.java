package com.cyborgcats.reusable.Autonomous;

public abstract class Odometer {
	private double tareX = 0.0, tareY = 0.0;
	protected ConsumableDouble x = new ConsumableDouble(), y = new ConsumableDouble();
	
	public abstract void init();
	public abstract void completeLoopUpdate();
	
	public void setOrigin(final double x, final double y) {tareX += x;	tareY += y;}
	
	public double getX(final boolean markAsRead) {return x.get(markAsRead) - tareX;}
	public double getY(final boolean markAsRead) {return y.get(markAsRead) - tareY;
}
	
	public boolean newX() {return x.isNew();}
	public boolean newY() {return y.isNew();}
	
	public static class ConsumableDouble {
		private boolean isNew = false;
		private double value = 0.0;
		
		public void set(final double value) {this.value = value;	isNew = true;}
		public void increment(final double value) {this.value += value;		isNew = true;}
		public double get(final boolean markAsRead) {if (markAsRead) isNew = false;		return value;}
		public boolean isNew() {return isNew;}
	}
}
