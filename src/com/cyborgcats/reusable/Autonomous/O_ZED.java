package com.cyborgcats.reusable.Autonomous;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;

public class O_ZED extends Odometer{
//	private static final double zedX = -0.75, zedY = 0.425;
	private final NetworkTable zed, position;
	private double tareX = 0.0, tareY = 0.0;
	private ConsumableDouble x = new ConsumableDouble();
	private ConsumableDouble y = new ConsumableDouble();
//	private final R_Gyro gyro;
	
//	private double gyroAngle = 0.0;
	
	public O_ZED(final NetworkTable zed/*, final R_Gyro gyro*/) {
		this.zed = zed;
		this.position = zed.getSubTable("Position");
		zed.getEntry("Enable Odometry").setBoolean(true);
//		this.gyro = gyro;
	}
	@Override
	public void init() {
		position.addEntryListener("X", (position, key, entry, value, flags) -> {this.onUpdatedX(value.getDouble());}, EntryListenerFlags.kUpdate);
		position.addEntryListener("Y", (position, key, entry, value, flags) -> {this.onUpdatedY(value.getDouble());}, EntryListenerFlags.kUpdate);
	}
	@Override
	public void setCurrent(final double x, final double y) {tareX = getX(false) - x;	tareY = getY(false) - y;}
	@Override
	public double getX(final boolean markAsRead) {return x.get(markAsRead) - tareX;}
	@Override
	public double getY(final boolean markAsRead) {return y.get(markAsRead) - tareY;}
	@Override
	public boolean newX() {return x.isNew();}
	@Override
	public boolean newY() {return y.isNew();}
	
	private void onUpdatedX(final double x) {this.x.set(x);}
	private void onUpdatedY(final double y) {this.y.set(y);}
//	private double zedX() {return zedX*Math.cos(gyroAngle) + zedY*Math.sin(gyroAngle);}
//	private double zedY() {return zedY*Math.cos(gyroAngle) + zedX*Math.sin(gyroAngle);}
//	public double getX(final boolean adjustForRotation) {return adjustForZED ? x.get() - tareX - zedX() : x.get();}
//	public double getY(final boolean adjustForRotation) {return adjustForZED ? y.get() - tareY - zedY() : y.get();}
	
	@Override
	public void update() {/*gyroAngle = Math.toRadians(gyro.getCurrentAngle());*/}
	@Override
	public void disable() {zed.getEntry("Enable Odometry").setBoolean(false);}
}
