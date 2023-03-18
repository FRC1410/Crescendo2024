package org.frc1410.framework.scheduler.task;

import org.frc1410.framework.scheduler.loop.Loop;
import org.frc1410.framework.scheduler.task.lock.TaskLock;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/**
 * Represents a task that is bound to a loop and being actively ticked. This class
 * acts as a manager over its held task and holds all of its runtime state.
 *
 * @see Task
 * @see Loop
 */
public record BoundTask(
		@NotNull LifecycleHandle handle,
		@NotNull Task job,
		@NotNull TaskPersistence persistence,
		@NotNull Observer observer,
		@Nullable TaskLock lock
) {
	public BoundTask {
		observer.init(handle); // Set the state to its correct initial value
	}
}