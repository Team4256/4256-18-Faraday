package org.usfirst.frc.team4256.robot;

public class Elevators {
	
	private R_ElevatorOne elevatorOne;
	private R_ElevatorTwo elevatorTwo;

	public Elevators (R_ElevatorOne elevatorOne, R_ElevatorTwo elevatorTwo) {
		
		this.elevatorOne = elevatorOne;
		this.elevatorTwo = elevatorTwo;
		
	}
	
	public void setInches(double firstStagePosition, double secondStagePosition, double desiredInches) {
		
		if(desiredInches > elevatorTwo.maximumHeight) {//stageOne needed
			elevatorTwo.setInches(elevatorTwo.maximumHeight);
			elevatorOne.setInches(desiredInches - elevatorTwo.maximumHeight);
		}else if (desiredInches < elevatorTwo.maximumHeight) {//stageOne not needed
			elevatorTwo.setInches(desiredInches);
			elevatorOne.setInches(0);
		}
	}
	

}
