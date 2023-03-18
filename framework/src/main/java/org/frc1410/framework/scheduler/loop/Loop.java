package org.frc1410.framework.scheduler.loop;

import org.frc1410.framework.phase.Phase;
import org.frc1410.framework.scheduler.task.BoundTask;
import org.frc1410.framework.scheduler.task.TaskScheduler;
import org.frc1410.framework.scheduler.task.TaskState;
import org.jetbrains.annotations.NotNull;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

/**
 * Loops hold bound tasks, and are ticked on their period. There is
 * also a default loop, that runs on the normal robot periodic tick
 * and is generally used for most tasks.
 *
 * <p>These loops are <i>not</i> the same as the loops found in 254's
 * code, for example, but they are similar. They do not get their own
 * thread, but they do handle execution of tasks.
 *
 * @see LoopStore#main
 */
public class Loop {

	private final TaskScheduler scheduler;
	private final Set<BoundTask> tasks = new HashSet<>();
	private final long period;
	private boolean disabled;

	public Loop(TaskScheduler scheduler, long period) {
		this.scheduler = scheduler;
		this.period = period;
	}

	public long getPeriod() {
		return period;
	}

	public double getPeriodSeconds() {
		return getPeriod() / 1000d;
	}

	public void add(BoundTask task) {
		tasks.add(task);
	}

	public void tick() {
		if (disabled) {
			return;
		}

		for (var task : Set.copyOf(tasks)) {
			if (task.handle().state == TaskState.TERMINATED) {
				tasks.remove(task);
				continue;
			}

			process(task);
		}
	}

	public void flagTransition(Phase newPhase) {
		disabled = newPhase == Phase.DISABLED;

		for (var task : tasks) {
			if (!task.persistence().shouldPersist(newPhase)) {
				task.handle().requestTermination();
			}
		}
	}

	public @NotNull Collection<@NotNull BoundTask> getTasks() {
		return tasks;
	}

	private void process(BoundTask task) {
		var job = task.job();
		var handle = task.handle();

		// Handle termination before ticking the observer.
		if (handle.state == TaskState.FLAGGED_TERMINATION) {
			job.end(true);
			handle.state = TaskState.TERMINATED;

			scheduler.lockHandler.releaseLocks(task);
			return;
		}

		// Tick the observer to update the task state.
		task.observer().tick(handle);

		// Just skip this iteration entirely if the task lock is claimed. This prevents fun things like
		// a case where an observer resumes execution of a task despite its lock being actively in use.
		if (!scheduler.lockHandler.ownsLocks(task)) {
			return;
		}

		switch (handle.state) {
			case FLAGGED_EXECUTION -> {
				job.init();

				handle.state = job.isFinished() ? TaskState.FLAGGED_COMPLETION : TaskState.EXECUTING;
			}

			case EXECUTING -> {
				job.execute();
				
				if (job.isFinished()) {
					handle.state = TaskState.FLAGGED_COMPLETION;
				}
			}

			case FLAGGED_COMPLETION -> {
				job.end(false);
				handle.state = TaskState.SUSPENDED;

				scheduler.lockHandler.releaseLocks(task);
			}

			case FLAGGED_SUSPENSION -> {
				job.end(true);
				handle.state = TaskState.SUSPENDED;

				scheduler.lockHandler.releaseLocks(task);
			}
		}
	}
}