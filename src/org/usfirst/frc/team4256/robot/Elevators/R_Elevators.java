package org.usfirst.frc.team4256.robot.Elevators;

import org.usfirst.frc.team4256.robot.R_Clamp;

public class R_Elevators {
	private static final double elevatorTwoClimbHeight = 8.0;
	private static final double elevatorOneHookHeight = 44.0;
	
	private R_ElevatorOne elevatorOne;
	private R_ElevatorTwo elevatorTwo;
	private double currentSetpoint = 0.0;//relative to clampy, not hook
	private boolean climbing = false;
	
//	private enum climbModeState {
//		Disabled,
//		Enabled;
//	}
	
//	private climbModeState currentClimbModeState = climbModeState.Disabled;

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
	
	private double validateInches(final double inches) {
		final double minimumHeight = climbing ? elevatorOneHookHeight : 0.0;
		final double maximumHeight = climbing ? elevatorOneHookHeight + R_ElevatorOne.maximumHeight : R_ElevatorOne.maximumHeight + R_ElevatorTwo.maximumHeight;
		return Math.min(Math.max(inches, minimumHeight), maximumHeight);//clips values to be between 0 and maximumHeight
	}
	
	public void increment(final double inches) {
		if (climbing) setInches(currentSetpoint + inches + elevatorOneHookHeight - elevatorTwo.getInches());
		else setInches(currentSetpoint + inches);
	}
	
	public void setInches(final double desiredInches) {
		if (climbing) setInches_climb(desiredInches);//desiredInches is relative to hook
		else setInches_normal(desiredInches);//desiredInches is relative to clampy
//		switch(currentClimbModeState) {
//		case Disabled:
//			setInches_normal(desiredInches);	
//		case Enabled:
//			setInches_climb(desiredInches);
//		}
	}
	
//	public double getInches() {
//		if (climbing) return currentSetpoint + elevatorOneHookHeight;
//		else return currentSetpoint;
//	}
	
	/**
	 * This function utilizes both elevators to move the elevator to a certain inch value while maintaining a good center of gravity
	**/
	private void setInches_normal(double desiredInches) {
		desiredInches = validateInches(desiredInches);
		
		final double currentElOnePosition = elevatorOne.getInches();
		final boolean desiredAboveCurrentElOne = desiredInches > currentElOnePosition;
		if (desiredAboveCurrentElOne) {
			elevatorOne.setTorque(false);
			if (desiredInches <= currentElOnePosition + R_ElevatorTwo.maximumHeight) {
				elevatorTwo.setInches(desiredInches - elevatorOne.getInches());
			}else {
				elevatorTwo.setInches(R_ElevatorTwo.maximumHeight);
				if (elevatorTwo.isThere(5.0)) elevatorOne.setInches(desiredInches - elevatorTwo.getInches());
			}
		}else {
			elevatorOne.setTorque(true);
			elevatorTwo.setInches(0.0);
			if (elevatorTwo.isThere(5.0)) elevatorOne.setInches(desiredInches);
		}
		currentSetpoint = desiredInches;
	}
	
	private void setInches_climb(double desiredInches) {
		desiredInches = validateInches(desiredInches);
		final double elevatorOneDesiredInches = desiredInches - elevatorOneHookHeight;
		elevatorOne.setInches(elevatorOneDesiredInches);
		currentSetpoint = elevatorOneDesiredInches + elevatorTwo.getInches();//getInches should result in approx. elevatorTwoClimbHeight, bc that's what enableClimbMode made it
	}
	
	public void enableClimbMode(final R_Clamp clamp) {
		elevatorTwo.setInches(elevatorTwoClimbHeight);
		clamp.close();
		clamp.retract();
		if (!elevatorOne.hasLotsOfTorque()) elevatorOne.setTorque(true);
		climbing = true;
		setInches(81.0);
	}
	
	public void disableClimbMode(final R_Clamp clamp) {
		clamp.extend();
		climbing = false;
	}
	
	public boolean inClimbingMode() {
		return climbing;
	}
	
//	public boolean readyToClimb(final R_Clamp clamp) {
//		if (currentClimbModeState == climbModeState.Enabling) {
//			if(elevatorTwo.isThere(2.0) && !clamp.isExtended() && !clamp.isOpen() && elevatorOne.hasLotsOfTorque()) {
//				currentClimbModeState = climbModeState.Enabled;
//			}
//		}
//		return currentClimbModeState == climbModeState.Enabled;
//	}
	
	public void completeLoopUpdate() {
		elevatorOne.completeLoopUpdate();
		elevatorTwo.completeLoopUpdate();
	}

}
