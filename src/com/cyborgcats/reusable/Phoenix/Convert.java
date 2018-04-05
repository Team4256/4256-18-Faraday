package com.cyborgcats.reusable.Phoenix;//COMPLETE 2018

public class Convert {
	private final double countsPerRev;
	private final double gearRatio;
	public final To to;
	public final From from;
	
	public Convert(final int countsPerRev, final double gearRatio) {
		this.countsPerRev = (double)countsPerRev;
		this.gearRatio = gearRatio;
		this.to = new To();
		this.from = new From();
	}
	
	private enum Unit {REVS_UNIT, DEGREES_UNIT, RPM_UNIT, RPS_UNIT;}

	public final class To {
		public final class Conversion {
			private final Unit unit;

			private Conversion(final Unit u) {unit = u;};
			
			public double beforeGears(final int encoderCounts) {
				switch(unit) {
				case REVS_UNIT: return encoderCounts/countsPerRev;
				case DEGREES_UNIT: return 360.0*encoderCounts/countsPerRev;
				case RPS_UNIT: return 10*encoderCounts/countsPerRev;
				case RPM_UNIT: return 60*10*encoderCounts/countsPerRev;
				default: return encoderCounts;
				}
			}
			
			public double afterGears(final int encoderCounts) {
				return beforeGears(encoderCounts)/gearRatio;
			}
		}

		public final Conversion REVS = new Conversion(Unit.REVS_UNIT);
		public final Conversion DEGREES = new Conversion(Unit.DEGREES_UNIT);
		public final Conversion RPS = new Conversion(Unit.RPS_UNIT);
		public final Conversion RPM = new Conversion(Unit.RPM_UNIT);
	}
	
	
	public final class From {
		public final class Conversion {
			private final Unit unit;
			
			private Conversion(final Unit u) {unit = u;};

			public double beforeGears(final double value) {
				switch(unit) {
				case REVS_UNIT: return value*countsPerRev;
				case DEGREES_UNIT: return value*countsPerRev/360.0;
				case RPS_UNIT: return value*countsPerRev/10;
				case RPM_UNIT: return value*countsPerRev/(10*60);
				default: return value;
				}
			}
		
			public double afterGears(final double value) {
				return beforeGears(value)*gearRatio;
			}
		}
		
		public final Conversion REVS = new Conversion(Unit.REVS_UNIT);
		public final Conversion DEGREES = new Conversion(Unit.DEGREES_UNIT);
		public final Conversion RPS = new Conversion(Unit.RPS_UNIT);
		public final Conversion RPM = new Conversion(Unit.RPM_UNIT);
	}
}
