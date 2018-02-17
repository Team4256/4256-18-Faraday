package org.usfirst.frc.team4256.robot.Elevators;

public class R_Elevators {
	
	private R_ElevatorOne elevatorOne;
	private R_ElevatorTwo elevatorTwo;

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
	

	
	public void completeLoopUpdate() {
		elevatorOne.completeLoopUpdate();
		elevatorTwo.completeLoopUpdate();
	}

}
