package org.frc1410.framework.phase;

public enum Phase {

	EMERGENCY_STOPPED,
	DISABLED,
	TEST,

	AUTONOMOUS,
	TELEOP,

	INIT,
	TRANSITION;

	public boolean isGameplay() {
		return this == AUTONOMOUS || this == TELEOP;
	}
}
