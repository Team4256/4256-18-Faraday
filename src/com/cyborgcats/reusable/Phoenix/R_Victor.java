package com.cyborgcats.reusable.Phoenix;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class R_Victor extends VictorSPX {
	
	public static final ControlMode current = ControlMode.Current;
	public static final ControlMode follower = ControlMode.Follower;
	public static final ControlMode percent = ControlMode.PercentOutput;
	public static final ControlMode position = ControlMode.Position;
	public static final ControlMode velocity = ControlMode.Velocity;
	public static final ControlMode disabled = ControlMode.Disabled;
	public static final NeutralMode brake = NeutralMode.Brake;
	public static final NeutralMode coast = NeutralMode.Coast;
	
	private ControlMode controlMode;
	public static final int kTimeoutMS = 10;
	private double lastSetPoint = 0.0;
	private boolean updated = false;
	
	public R_Victor(int deviceID, final ControlMode controlMode) {
		super(deviceID);
		this.controlMode = controlMode;
	}
	
	public void init(final int masterID, final double maxPercent) {
		clearStickyFaults(kTimeoutMS);
		configNominalOutputForward(0.0, kTimeoutMS);
		configNominalOutputReverse(0.0, kTimeoutMS);
		configPeakOutputForward(Math.abs(maxPercent), kTimeoutMS);
		configPeakOutputReverse(-Math.abs(maxPercent), kTimeoutMS);
		
		if (getControlMode() == follower) {
			quickSet(masterID);
		}else {
			quickSet(0.0);
		}
	}
	
	public void init() {
		init(0, 1.0);
	}
	
	public void quickSet(final double value) {
		try {
			this.set(value, true);
		} catch (IllegalAccessException e) {
			e.printStackTrace();
		}
	}
	
	public void set(final double value, final boolean updateSetPoint) throws IllegalAccessException {
		double currentSetPoint = lastSetPoint;
		switch (controlMode) {
		case Current:
			throw new IllegalAccessException("Victor " + Integer.toString(getDeviceID()) + "'s mode is not compatible with Victors");
		case Follower:
			break;
		case PercentOutput:
			break;
		case Position:
			throw new IllegalAccessException("Victor " + Integer.toString(getDeviceID()) + "'s mode is not compatible with Victors");
		case Velocity:
			throw new IllegalAccessException("Victor " + Integer.toString(getDeviceID()) + "'s mode is not compatible with Victors");
		case Disabled:
			break;
		default:throw new IllegalAccessException("Victor " + Integer.toString(getDeviceID()) + "'s mode is unimplemented.");
		}
		
		updated = true;
		if (updateSetPoint) {
			lastSetPoint = currentSetPoint;
		}
	}
	
	private double setPercent(final double percentage) throws IllegalAccessException {
		if (controlMode == percent) {
			super.set(controlMode, percentage);
		}else {
			throw new IllegalAccessException("Victor " + Integer.toString(getDeviceID()) + " was given percentage in " + controlMode.name() + " mode.");
		}return percentage;
	}
	
	public void completeLoopUpdate() {
		if (!updated) {super.set(controlMode, lastSetPoint);}//send a command if there hasn't yet been one, using raw encoder units
		
		if (getControlMode() != follower) {updated = false;}//loop is over, reset updated for use in next loop (followers excluded)
	}
	

}
