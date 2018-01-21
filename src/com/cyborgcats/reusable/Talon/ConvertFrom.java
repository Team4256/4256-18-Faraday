package com.cyborgcats.reusable.Talon;

public enum ConvertFrom {
	REVS,
	DEGREES;
	
	public static final double countsPerRev = 4096.0;
	
	double beforeGears(final double value) {
		switch(this) {
		case REVS: return value*countsPerRev;
		case DEGREES: return value*countsPerRev/360.0;
		default: return value;
		}
	}
	
	double afterGears(final double gearRatio, final double value) {
		return beforeGears(value)*gearRatio;
	}
}
