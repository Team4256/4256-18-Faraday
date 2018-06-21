package org.usfirst.frc.team4256.robot.Elevators;

import org.usfirst.frc.team4256.robot.Clamp;

import com.cyborgcats.reusable.Subsystem;

public class R_Combined implements Subsystem {
	private static final double initialClimbingHeight = 81.0;//inches
	private E_One one;
	private E_Two two;
	private double currentSetpoint = 0.0;//should always be the distance from 0 to clamp, not 0 to hook
	private boolean climbing = false;

	public R_Combined(E_One one, E_Two two) {
		this.one = one;
		this.two = two;
	}
	
	/**
	 * This function prepares each elevator individually.
	**/
	public void init() {
		one.init();
		two.init();
	}
	
	
//	public double getInches() {//TODO
//	if (climbing) return currentSetpoint + E_One.hookHeight;
//	else return currentSetpoint;
//}
	
	
	/**
	 * This function ensures requested heights are within the abilites of the elevators in their current mode.
	 * In climbing mode, minimum is hookBaseline and maximum is (hookBaseline + El1.maxHeight).
	 * Outside climbing mode, minimum is 0 and maximum is (El1.maxHeight + El2.maxHeight).
	**/
	private double validateInches(final double inches) {
		final double minimumHeight = climbing ? E_One.hookBaseline : 0.1;//ideally 0, but .1 helps avoid encoder noise around 0
		final double maximumHeight = climbing ? E_One.hookBaseline + E_One.maximumHeight : E_One.maximumHeight + E_Two.maximumHeight;
		return Math.min(Math.max(inches, minimumHeight), maximumHeight);//clips values outside of [minimumHeight, maximumHeight]
	}
	
	
	public void increment(final double inches) {
		if (climbing) setInches(currentSetpoint + inches + E_One.hookBaseline - two.getInches());
		else setInches(currentSetpoint + inches);
	}
	
	
	/**
	 * This function calls designated set functions depending on whether climbing is enabled.
	 * In climbing mode, desiredInches will become hook height.
	 * Outside climbing mode, desiredInches will become clamp height.
	**/
	public void setInches(final double desiredInches) {
		if (climbing) setHookHeight(desiredInches);
		else setClampHeight(desiredInches);
	}
	
	
	/**
	 * This function moves the clamp to desiredInches, prioritizing EChain preservation over speed and CG.
	 * Basically, this means that elevatorTwo will reach its limit before elevatorOne begins moving.
	**/
	private void setClampHeight(double desiredInches) {
		desiredInches = validateInches(desiredInches);
		
		final double currentElOnePosition = one.getInches();
		if (desiredInches > currentElOnePosition) {
			one.setTorque(false);//when moving up, go really fast
			if (desiredInches <= currentElOnePosition + E_Two.maximumHeight) {
				two.setInches(desiredInches - one.getInches());
			}else {
				two.setInches(E_Two.maximumHeight);
				if (two.isThere(5.0)) one.setInches(desiredInches - two.getInches());
			}
		}else {
			one.setTorque(true);//when coming down, slow down a bit
			two.setInches(0.0);
			if (two.isThere(5.0)) one.setInches(desiredInches);
		}
		currentSetpoint = desiredInches;
	}
	
	
	/**
	 * This function moves the hooks to desiredInches, utilizing only elevatorOne.
	 * Ignores EChain placement.
	**/
	private void setHookHeight(double desiredInches) {
		desiredInches = validateInches(desiredInches);
		final double elevatorOneDesiredInches = desiredInches - E_One.hookBaseline;
		one.setInches(elevatorOneDesiredInches);
		currentSetpoint = elevatorOneDesiredInches + two.getInches();//getInches should result in approx. E_Two.climbingHeight, bc that's what enableClimbMode made it
	}
	
	
	public void enableClimbMode(final Clamp clamp) {
		two.setInches(E_Two.climbingHeight);
		clamp.close();
		clamp.rotateTo(90.0);
		one.setTorque(true);
		climbing = true;
		setInches(initialClimbingHeight);
	}
	
	
	public void disableClimbMode() {climbing = false;}
	public boolean inClimbingMode() {return climbing;}
	
	
	public void completeLoopUpdate() {
		one.completeLoopUpdate();
		two.completeLoopUpdate();
	}

	@Override
	public boolean perform(String action, double[] data) {
		switch(Abilities.valueOf(action)) {
		case SET: setInches(data[0]); break;
		case INCREMENT: increment(data[0]); break;
		default: throw new IllegalStateException("The elevator cannot " + action);
		}
		return (one.isThere(2.0) && two.isThere(2.0));
	}

	public static enum Abilities {SET, INCREMENT}
}
