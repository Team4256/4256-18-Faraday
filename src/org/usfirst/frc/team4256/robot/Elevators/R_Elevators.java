package org.usfirst.frc.team4256.robot.Elevators;

public class R_Elevators {
	
	private R_ElevatorOne elevatorOne;
	private R_ElevatorTwo elevatorTwo;

	public R_Elevators (R_ElevatorOne elevatorOne, R_ElevatorTwo elevatorTwo) {
		this.elevatorOne = elevatorOne;
		this.elevatorTwo = elevatorTwo;
	}
	
	/**
	 * This function utilizes both elevators to move the elevator to a certain inch value while maintaining a good center of gravity
	**/
	public void setInches(final double desiredInches) {
		if(!elevatorOne.isLowGear()) {
			elevatorOne.shiftLowGear();
		}
		if(desiredInches > R_ElevatorTwo.maximumHeight) {//stageOne needed
			elevatorTwo.setInches(R_ElevatorTwo.maximumHeight);
			elevatorOne.setInches(desiredInches - R_ElevatorTwo.maximumHeight);
		}else if (desiredInches <= R_ElevatorTwo.maximumHeight) {//stageOne not needed
			elevatorTwo.setInches(desiredInches);
			elevatorOne.setInches(0);
		}
	}
	
	public void init() {
		elevatorOne.init();
		elevatorTwo.init();
	}
	
	public void completeLoopUpdate() {
		elevatorOne.completeLoopUpdate();
		elevatorTwo.completeLoopUpdate();
	}

}
