package com.cyborgcats.reusable.Talon;

public enum ConvertFrom {
	REVS,
	DEGREES;
	
	public double countsPerRev;
	
	public double beforeGears(final double value) {
		switch(this) {
		case REVS: return value*countsPerRev;
		case DEGREES: return value*countsPerRev/360.0;
		default: return value;
		}
	}
	
	public double afterGears(final double gearRatio, final double value) {
		return beforeGears(value)*gearRatio;
	}
}
