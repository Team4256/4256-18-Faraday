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
		if (desiredInches > R_ElevatorTwo.maximumHeight) {//stage one needed
			elevatorTwo.setInches(R_ElevatorTwo.maximumHeight);
			//make sure stage two is all the way up before adjusting stage one
			if (elevatorTwo.isThere(5.0)) elevatorOne.setInches(desiredInches - R_ElevatorTwo.maximumHeight);
		}else if (desiredInches <= R_ElevatorTwo.maximumHeight) {//stage one not needed
			//if stage one is up, make sure stage two is all the way down before adjusting it
			if (elevatorOne.getInches() >= 2.0) elevatorTwo.setInches(0.0);
			//after stage two is all the way down, move stage one all the way down
			if (elevatorTwo.isThere(2.0)) elevatorOne.setInches(0.0);//TODO could possible set stage two to desired position in this if statement
			//after stage one is all the way down, move stage two to desired position
			if (elevatorOne.getInches() < 2.0) elevatorTwo.setInches(desiredInches);
		}
	}
	

	
	public void completeLoopUpdate() {
		elevatorOne.completeLoopUpdate();
		elevatorTwo.completeLoopUpdate();
	}

}
