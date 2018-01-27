package com.cyborgcats.reusable.Talon;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public enum R_Encoder {
	CTRE_MAG_ABSOLUTE(FeedbackDevice.CTRE_MagEncoder_Absolute, 4096),
	CTRE_MAG_RELATIVE(FeedbackDevice.CTRE_MagEncoder_Relative, 4096),
	THE_FIRST_QUAD(FeedbackDevice.QuadEncoder, 12),
	THE_SECOND_QUAD(FeedbackDevice.QuadEncoder, 1111);
	
	private final FeedbackDevice feedbackDevice;
	private final int countsPerRev;
	
	R_Encoder(final FeedbackDevice feedbackDevice, final int countsPerRev) {
		this.feedbackDevice = feedbackDevice;
		this.countsPerRev = countsPerRev;
	}
	
	public FeedbackDevice type() {
		return feedbackDevice;
	}
	
	public int countsPerRev() {
		return countsPerRev;
	}
}
