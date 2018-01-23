package com.cyborgcats.reusable.Talon;

public enum ConvertTo {
	REVS,
	DEGREES;
	
	public static final double countsPerRev = 4096.0;
	
	public double beforeGears(final int encoderCounts) {
		switch(this) {
		case REVS: return encoderCounts/countsPerRev;
		case DEGREES: return 360.0*encoderCounts/countsPerRev;
		default: return encoderCounts;
		}
	}
	
	public double afterGears(final double gearRatio, final int encoderCounts) {
		return beforeGears(encoderCounts)/gearRatio;
	}
}
