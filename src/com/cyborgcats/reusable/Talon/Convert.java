package com.cyborgcats.reusable.Talon;

public class Convert {
	private double countsPerRev;
	private double gearRatio;
	public To to;
	public From from;
	
	public Convert(final int countsPerRev, final double gearRatio) {
		this.countsPerRev = (double)countsPerRev;
		this.gearRatio = gearRatio;
		this.to = new To();
		this.from = new From();
	}
	
	public enum Unit {
	REVS_UNIT,
	DEGREES_UNIT,
	RPM_UNIT;
	}

	public class To {
		public class Conversion {
			public Unit unit;

			public Conversion(Unit u) {unit = u;};
			
			public double beforeGears(final int encoderCounts) {
				switch(unit) {
				case REVS_UNIT: return encoderCounts/countsPerRev;
				case DEGREES_UNIT: return 360.0*encoderCounts/countsPerRev;
				case RPM_UNIT: return encoderCounts/countsPerRev;
				default: return encoderCounts;
				}
			}
			
			public double afterGears(final int encoderCounts) {
				return beforeGears(encoderCounts)/gearRatio;
			}
		}

		public Conversion REVS = new Conversion(Unit.REVS_UNIT);
		public Conversion DEGREES = new Conversion(Unit.DEGREES_UNIT);
		public Conversion RPM = new Conversion(Unit.RPM_UNIT);
	}
	
	
	public class From {
		public class Conversion {
			public Unit unit;
			
			public Conversion(Unit u) {unit = u;};

			public double beforeGears(final double value) {
				switch(unit) {
				case REVS_UNIT: return value*countsPerRev;
				case DEGREES_UNIT: return value*countsPerRev/360.0;
				case RPM_UNIT: return value*countsPerRev;
				default: return value;
				}
			}
		
			public double afterGears(final double value) {
				return beforeGears(value)*gearRatio;
			}
		}
		
		public Conversion REVS = new Conversion(Unit.REVS_UNIT);
		public Conversion DEGREES = new Conversion(Unit.DEGREES_UNIT);
		public Conversion RPM = new Conversion(Unit.RPM_UNIT);
	}
}
