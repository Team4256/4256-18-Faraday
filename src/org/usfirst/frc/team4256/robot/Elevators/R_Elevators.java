package org.usfirst.frc.team4256.robot.Elevators;

import org.usfirst.frc.team4256.robot.R_Clamp;

public class R_Elevators {
	
	private R_ElevatorOne elevatorOne;
	private R_ElevatorTwo elevatorTwo;
	
	private enum climbModeState {
		Disabled,
		Enabling,
		Enabled
	}
	
	private climbModeState currentClimbModeState = climbModeState.Disabled;

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
	
	
	/**
	 * This function utilizes both elevators to move the elevator to a certain inch value while maintaining a good center of gravity
	**/
	public void setInches(final double desiredInches) {
		if (elevatorOne.hasLotsOfTorque()) {
			elevatorOne.setTorque(false);//TODO change PID parameters depending on which one we are using
		}
		
		final double currentElOnePosition = elevatorOne.getInches();
		final boolean desiredAboveCurrentElOne = desiredInches > currentElOnePosition;
		if (desiredAboveCurrentElOne) {
			if (desiredInches <= currentElOnePosition + R_ElevatorTwo.maximumHeight) {
				elevatorTwo.setInches(desiredInches - elevatorOne.getInches());
			}else {
				elevatorTwo.setInches(R_ElevatorTwo.maximumHeight);
				if (elevatorTwo.isThere(5.0)) elevatorOne.setInches(desiredInches - elevatorTwo.getInches());
			}
		}else {
			elevatorTwo.setInches(0.0);
			if (elevatorTwo.isThere(5.0)) elevatorOne.setInches(desiredInches);
		}
	}
	
	public void setInchesClimb(final double desiredInches) {
		elevatorOne.setInches(desiredInches);
	}
	
	public void enableClimbMode(final R_Clamp clamp) {
		elevatorTwo.setInches(8.0);
		clamp.close();
		clamp.retract();
		if (!elevatorOne.hasLotsOfTorque()) elevatorOne.setTorque(true);
		currentClimbModeState = climbModeState.Enabling;
	}
	
	public boolean readyToClimb(final R_Clamp clamp) {
		if (currentClimbModeState == climbModeState.Enabling) {
			if(Math.round(elevatorTwo.getInches()) == 8.0  && !clamp.isExtended() && !clamp.isOpen() && elevatorOne.hasLotsOfTorque()) {
				currentClimbModeState = climbModeState.Enabled;
			}
		}
		return currentClimbModeState == climbModeState.Enabled;
	}
	
	public void completeLoopUpdate() {
		elevatorOne.completeLoopUpdate();
		elevatorTwo.completeLoopUpdate();
	}

}
