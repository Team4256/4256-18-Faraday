package org.usfirst.frc.team4256.robot.Elevators;

import org.usfirst.frc.team4256.robot.R_Clamp;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class R_Elevators {
	
	public boolean isClimbing = false;
	
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
		SmartDashboard.putNumber("el 1", elevatorOne.getInches());
		SmartDashboard.putNumber("el 2", elevatorTwo.getInches());
		if (desiredInches > R_ElevatorTwo.maximumHeight) {//stage one needed
			elevatorTwo.setInches(R_ElevatorTwo.maximumHeight);
			elevatorOne.setInches(desiredInches - R_ElevatorTwo.maximumHeight);
		}else if (desiredInches <= R_ElevatorTwo.maximumHeight) {//stage one not needed
			elevatorTwo.setInches(desiredInches);
			elevatorOne.setInches(0.0);
			//if stage one is up, make sure stage two is all the way down before adjusting it
//			if (elevatorOne.getInches() >= 2.0) elevatorTwo.setInches(0.0);
			//after stage two is all the way down, move stage one all the way down
//			if (elevatorTwo.getInches() <= 2.0) elevatorOne.setInches(0.0);//TODO could possibly set stage two to desired position in this if statement
			//after stage one is all the way down, move stage two to desired position
//			if (elevatorOne.getInches() < 2.0) elevatorTwo.setInches(desiredInches);
		}
	}
	
	public void climb(final R_Clamp clamp) {
		if (!elevatorOne.hasLotsOfTorque()) elevatorOne.setTorque(true);
		elevatorTwo.setInches(8.0);
		clamp.close();
		clamp.retract();
		if (elevatorTwo.isThere(1.0) && !clamp.isExtended() && !clamp.isOpen()) elevatorOne.setInches(0.0);
	}
	
	public boolean isClimbing() {
		return isClimbing;
	}
	
	public void completeLoopUpdate() {
		elevatorOne.completeLoopUpdate();
		elevatorTwo.completeLoopUpdate();
	}

}
