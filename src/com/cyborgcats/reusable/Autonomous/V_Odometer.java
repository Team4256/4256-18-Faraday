package com.cyborgcats.reusable.Autonomous;//TODO could be an interface

import com.cyborgcats.reusable.R_Gyro;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;

public class V_Odometer {
	private static final double zedX = -0.75, zedY = 0.425;
	private final NetworkTable zed, position;
	private double tareX = 0.0, tareY = 0.0;
	private ConsumableDouble x = new ConsumableDouble();
	private ConsumableDouble y = new ConsumableDouble();
	private final R_Gyro gyro;
	
	private double gyroAngle = 0.0;
	
	public V_Odometer(final NetworkTable zed, final R_Gyro gyro) {
		this.zed = zed;
		this.position = zed.getSubTable("Position");
		zed.getEntry("Enable Odometry").setBoolean(true);
		this.gyro = gyro;
	}
	
	public void init() {
		position.addEntryListener("X", (position, key, entry, value, flags) -> {this.onUpdatedX(value.getDouble());}, EntryListenerFlags.kUpdate);
		position.addEntryListener("Y", (position, key, entry, value, flags) -> {this.onUpdatedY(value.getDouble());}, EntryListenerFlags.kUpdate);
	}
	
	public void setOrigin(final double x, final double y) {tareX = x;	tareY = y;}
	
	public double getX(final boolean adjustForZED) {return adjustForZED ? x.get() - tareX - zedX() : x.get();}
	public double getY(final boolean adjustForZED) {return adjustForZED ? y.get() - tareY - zedY() : y.get();}
	public boolean newX() {return x.isNew();}
	public boolean newY() {return y.isNew();}
	
	private void onUpdatedX(final double x) {this.x.set(x);}
	private void onUpdatedY(final double y) {this.y.set(y);}
	private double zedX() {return zedX*Math.cos(gyroAngle) + zedY*Math.sin(gyroAngle);}
	private double zedY() {return zedY*Math.cos(gyroAngle) + zedX*Math.sin(gyroAngle);}
	
	private class ConsumableDouble {
		private boolean isNew = false;
		private double value = 0.0;
		
		public void set(final double value) {this.value = value;	isNew = true;}
		public double get() {isNew = false;		return value;}
		public boolean isNew() {return isNew;}
	}
	
	public void update() {gyroAngle = Math.toRadians(gyro.getCurrentAngle());}
	public void disable() {zed.getEntry("Enable Odometry").setBoolean(false);}
}
