package org.frc1410.framework.scheduler.task;

import org.jetbrains.annotations.NotNull;

/**
 * This interface is responsible for linking tasks to external conditions such
 * as I/O inputs. Observers are ticked every time their parent task is ticked,
 * and can update the task's state using its {@link LifecycleHandle}.
 */
@FunctionalInterface
public interface Observer {

	/**
	 * The default observer that requests execution every tick.
	 */
	Observer DEFAULT = LifecycleHandle::requestExecution;

	/**
	 * An observer that will not modify the task's state at all.
	 */
	Observer NO_OP = new NoOpObserver();

	void tick(@NotNull LifecycleHandle handle);

	default void init(LifecycleHandle handle) {

	}
}

final class NoOpObserver implements Observer {

	@Override
	public void tick(@NotNull LifecycleHandle handle) {

	}

	@Override
	public void init(LifecycleHandle handle) {
		handle.requestExecution();
	}
}