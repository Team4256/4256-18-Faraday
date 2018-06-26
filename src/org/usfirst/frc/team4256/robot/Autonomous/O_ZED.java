package org.usfirst.frc.team4256.robot.Autonomous;

import com.cyborgcats.reusable.Autonomous.Odometer;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;

public class O_ZED extends Odometer{
	private final NetworkTable zed, position;
	
	public O_ZED(final NetworkTable zed) {
		this.zed = zed;
		this.position = zed.getSubTable("Position");
		zed.getEntry("Enable Odometry").setBoolean(true);
	}
	
	private void onUpdatedX(final double x) {this.x.set(x);}
	private void onUpdatedY(final double y) {this.y.set(y);}
	
	public void disable() {zed.getEntry("Enable Odometry").setBoolean(false);}
	
	@Override
	public void init() {
		position.addEntryListener("X", (position, key, entry, value, flags) -> {this.onUpdatedX(value.getDouble());}, EntryListenerFlags.kUpdate);
		position.addEntryListener("Y", (position, key, entry, value, flags) -> {this.onUpdatedY(value.getDouble());}, EntryListenerFlags.kUpdate);
	}
	@Override
	public void completeLoopUpdate() {}
}
