package org.usfirst.frc.team4256.robot.Elevators;

public class R_Elevators {
	
	private R_ElevatorOne elevatorOne;
	private R_ElevatorTwo elevatorTwo;

	public R_Elevators (R_ElevatorOne elevatorOne, R_ElevatorTwo elevatorTwo) {
		this.elevatorOne = elevatorOne;
		this.elevatorTwo = elevatorTwo;
	}
	
	public void setInches(final double desiredInches) {
		
		if(desiredInches > R_ElevatorTwo.maximumHeight) {//stageOne needed
			elevatorTwo.setInches(R_ElevatorTwo.maximumHeight);
			elevatorOne.setInches(desiredInches - R_ElevatorTwo.maximumHeight);
		}else if (desiredInches <= R_ElevatorTwo.maximumHeight) {//stageOne not needed
			elevatorTwo.setInches(desiredInches);
			elevatorOne.setInches(0);
		}
	}
	

}
