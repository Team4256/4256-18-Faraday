package org.usfirst.frc.team4256.robot.Autonomous;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;

public class V_Odometer {
	private final NetworkTable zed;
	private double tareX = 0.0, tareY = 0.0;
	private ConsumableDouble x, y;
	
	public V_Odometer(final NetworkTable zed) {
		this.zed = zed;
	}
	
	public void init() {
		zed.addEntryListener("X", (zed, key, entry, value, flags) -> {onUpdatedX(value.getDouble());}, EntryListenerFlags.kUpdate);
		zed.addEntryListener("Y", (zed, key, entry, value, flags) -> {onUpdatedY(value.getDouble());}, EntryListenerFlags.kUpdate);
	}
	
	public void setOrigin(final double x, final double y) {tareX = x;	tareY = y;}
	
	public double getX() {return x.get() - tareX;}
	public double getY() {return y.get() - tareY;}
	public boolean newX() {return x.isNew();}
	public boolean newY() {return y.isNew();}
	
	private void onUpdatedX(final double x) {this.x.set(x);}
	private void onUpdatedY(final double y) {this.y.set(y);}
	
	private class ConsumableDouble {
		private boolean isNew = false;
		private double value;
		
		public void set(final double value) {this.value = value;	isNew = true;}
		public double get() {isNew = false;		return value;}
		public boolean isNew() {return isNew;}
	}
}
