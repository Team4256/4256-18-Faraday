package org.usfirst.frc.team4256.robot.Elevators;

import org.usfirst.frc.team4256.robot.R_Clamp;

public class R_Elevators {
	private static final double initialClimbingHeight = 81.0;//inches
	private R_ElevatorOne elevatorOne;
	private R_ElevatorTwo elevatorTwo;
	private double currentSetpoint = 0.0;//should always be the distance from 0 to clamp, not 0 to hook
	private boolean climbing = false;

	public R_Elevators (R_ElevatorOne elevatorOne, R_ElevatorTwo elevatorTwo) {
		this.elevatorOne = elevatorOne;
		this.elevatorTwo = elevatorTwo;
	}
	
	/**
	 * This function prepares each elevator individually.
	**/
	public void init() {
		elevatorOne.init();
		elevatorTwo.init();
	}
	
	
//	public double getInches() {
//	if (climbing) return currentSetpoint + R_ElevatorOne.hookHeight;
//	else return currentSetpoint;
//}
	
	
	/**
	 * This function ensures requested heights are within the abilites of the elevators in their current mode.
	 * In climbing mode, minimum is hookBaseline and maximum is (hookBaseline + El1.maxHeight).
	 * Outside climbing mode, minimum is 0 and maximum is (El1.maxHeight + El2.maxHeight).
	**/
	private double validateInches(final double inches) {
		final double minimumHeight = climbing ? R_ElevatorOne.hookBaseline : 0.0;
		final double maximumHeight = climbing ? R_ElevatorOne.hookBaseline + R_ElevatorOne.maximumHeight : R_ElevatorOne.maximumHeight + R_ElevatorTwo.maximumHeight;
		return Math.min(Math.max(inches, minimumHeight), maximumHeight);//clips values outside of [minimumHeight, maximumHeight]
	}
	
	
	public void increment(final double inches) {
		if (climbing) setInches(currentSetpoint + inches + R_ElevatorOne.hookBaseline - elevatorTwo.getInches());
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
	 * Basically, this means that elevatorTwo will reach its limit
	 * in the direction of motion before elevatorOne begins moving.
	**/
	private void setClampHeight(double desiredInches) {
		desiredInches = validateInches(desiredInches);
		
		final double currentElOnePosition = elevatorOne.getInches();
		final boolean desiredAboveCurrentElOne = desiredInches > currentElOnePosition;
		if (desiredAboveCurrentElOne) {
			elevatorOne.setTorque(false);//when moving up, go really fast
			if (desiredInches <= currentElOnePosition + R_ElevatorTwo.maximumHeight) {
				elevatorTwo.setInches(desiredInches - elevatorOne.getInches());
			}else {
				elevatorTwo.setInches(R_ElevatorTwo.maximumHeight);
				if (elevatorTwo.isThere(5.0)) elevatorOne.setInches(desiredInches - elevatorTwo.getInches());
			}
		}else {
			elevatorOne.setTorque(true);//when coming down, slow down a bit
			elevatorTwo.setInches(0.0);
			if (elevatorTwo.isThere(5.0)) elevatorOne.setInches(desiredInches);
		}
		currentSetpoint = desiredInches;
	}
	
	
	/**
	 * This function moves the hooks to desiredInches, utilizing only elevatorOne.
	 * Ignores EChain placement.//TODO?
	**/
	private void setHookHeight(double desiredInches) {
		desiredInches = validateInches(desiredInches);
		final double elevatorOneDesiredInches = desiredInches - R_ElevatorOne.hookBaseline;
		elevatorOne.setInches(elevatorOneDesiredInches);
		currentSetpoint = elevatorOneDesiredInches + elevatorTwo.getInches();//getInches should result in approx. R_ElevatorTwo.climbingHeight, bc that's what enableClimbMode made it
	}
	
	
	public void enableClimbMode(final R_Clamp clamp) {
		elevatorTwo.setInches(R_ElevatorTwo.climbingHeight);
		clamp.close();
		clamp.retract();
		elevatorOne.setTorque(true);
		climbing = true;
		setInches(initialClimbingHeight);
	}
	
	
	public void disableClimbMode(final R_Clamp clamp) {
		clamp.extend();
		climbing = false;
	}
	
	
	public boolean inClimbingMode() {
		return climbing;
	}
	
	
	public void completeLoopUpdate() {
		elevatorOne.completeLoopUpdate();
		elevatorTwo.completeLoopUpdate();
	}

}
