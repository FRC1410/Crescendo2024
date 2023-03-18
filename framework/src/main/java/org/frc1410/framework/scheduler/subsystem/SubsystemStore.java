package org.frc1410.framework.scheduler.subsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frc1410.framework.scheduler.task.TaskPersistence;
import org.frc1410.framework.scheduler.task.TaskScheduler;
import org.frc1410.framework.scheduler.task.impl.SubsystemPeriodicTask;
import org.frc1410.framework.scheduler.task.lock.LockPriority;
import org.frc1410.framework.scheduler.task.Observer;
import org.frc1410.framework.util.log.Logger;
import org.jetbrains.annotations.NotNull;

import java.util.Objects;

public final class SubsystemStore {

	private static final Logger LOG = new Logger("SubsystemStore");

	private final TaskScheduler scheduler;

	public SubsystemStore(@NotNull TaskScheduler scheduler) {
		this.scheduler = Objects.requireNonNull(scheduler);
	}

	public <S extends Subsystem> S track(S subsystem) {
		if (subsystem instanceof TickedSubsystem ticked) {
			var task = new SubsystemPeriodicTask(ticked);
			var period = ticked.getPeriod();

			LOG.info("Registered subsystem %s for ticking with period %d", subsystem, period);
			if (period != -1L) {
				scheduler.schedule(task, TaskPersistence.DURABLE, Observer.DEFAULT, LockPriority.NULL, period);
			} else {
				scheduler.schedule(task, TaskPersistence.DURABLE, Observer.DEFAULT, LockPriority.NULL);
			}
		} else {
			LOG.warn("Registered subsystem %s but it is not ticked so it will not be scheduled.", subsystem);
		}
		
		return subsystem;
	}
}