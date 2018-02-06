package org.usfirst.frc.team4256.robot;

import com.cyborgcats.reusable.Talon.R_Encoder;
import com.cyborgcats.reusable.Talon.R_Talon;

import edu.wpi.first.wpilibj.DigitalInput;

public class R_ElevatorTwo {
	private static final double gearRatio = 1.0;
	private static final double sprocketCircumference = 1.29*Math.PI;//inches
	private R_Talon master;
	private DigitalInput limitSwitch;
	private boolean knowsZero = false;
	
	public R_ElevatorTwo(final int masterID, final int limitSwitchPort) {
		master = new R_Talon(masterID, gearRatio, R_Talon.position, R_Encoder.OEM_QUAD, false);
		limitSwitch = new DigitalInput(limitSwitchPort);
	}
	
	/**
	 * 
	**/
	public void init() {
		master.setNeutralMode(R_Talon.coast);//TODO which works better?
		master.init();
	}
	
	/**
	 *
	**/
	public void zero() {
		if(!limitSwitch.get()) {
			knowsZero = false;
			master.quickSet(-0.3, false);
		}else {
			master.quickSet(0, false);
			master.getSensorCollection().setQuadraturePosition(0, R_Talon.kTimeoutMS);
			knowsZero = true;
		}
	}
}
