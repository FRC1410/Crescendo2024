package org.frc1410.framework.scheduler.task;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.framework.scheduler.task.impl.CommandTask;

import java.util.function.Supplier;

public final class DeferredTask implements Task {

	private final TaskScheduler scheduler;
	private final Supplier<Task> taskSupplier;
	private final TaskPersistence persistence;
	private final Observer observer;
	private final int lockPriority;
	private BoundTask task;

	public DeferredTask(TaskScheduler scheduler, Supplier<Task> taskSupplier, TaskPersistence persistence, Observer observer, int lockPriority) {
		this.scheduler = scheduler;
		this.taskSupplier = taskSupplier;
		this.persistence = persistence;
		this.observer = observer;
		this.lockPriority = lockPriority;
	}

	public static DeferredTask fromCommand(TaskScheduler scheduler, Supplier<Command> commandSupplier) {
		return new DeferredTask(scheduler, () -> new CommandTask(commandSupplier.get()), TaskPersistence.EPHEMERAL, Observer.NO_OP, 4);
	}

	@Override
	public void init() {
		var job = taskSupplier.get();
		this.task = scheduler.schedule(job, persistence, observer, lockPriority);
	}

	@Override
	public boolean isFinished() {
		return task.handle().state.isInactive();
	}

	@Override
	public void end(boolean interrupted) {
		if (task != null) task.handle().requestTermination();
		task = null;
	}
}