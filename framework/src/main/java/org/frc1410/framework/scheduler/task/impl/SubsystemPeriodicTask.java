package org.frc1410.framework.scheduler.task.impl;

import edu.wpi.first.wpilibj.RobotBase;
import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;
import org.frc1410.framework.scheduler.task.Task;

/**
 * A wrapper task around {@link TickedSubsystem}s to call their
 * {@link TickedSubsystem#periodic()} hooks.
 */
public final class SubsystemPeriodicTask implements Task {

	private final TickedSubsystem subsystem;

	public SubsystemPeriodicTask(TickedSubsystem subsystem) {
		this.subsystem = subsystem;
	}

	@Override
	public void execute() {
		subsystem.periodic();

		if (RobotBase.isSimulation()) {
			subsystem.simulationPeriodic();
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}