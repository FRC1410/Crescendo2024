package org.frc1410.framework.scheduler.subsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface TickedSubsystem extends Subsystem {

	default long getPeriod() {
		return -1L;
	}

	@Override
	void periodic();

	@Override
	default void simulationPeriodic() {
		
	}
}