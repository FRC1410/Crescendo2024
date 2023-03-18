package org.frc1410.framework.scheduler.task;

import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.Set;

/**
 * Tasks are the base unit of execution for the {@link TaskScheduler}.
 * They have hooks for initialization, execution, and completion. They
 * also contain locks, which control access to in-demand resources for
 * cases where they cannot be accessed concurrently.
 *
 * <p>One task object can progress through its entire lifecycle multiple
 * times. This means that state should not be set up in constructors but
 * instead in {@link Task#init()}.
 *
 * <p>The scheduler does not execute tasks directly, but rather builds
 * them into {@link BoundTask}s.
 */
public interface Task {

	/**
	 * The hook called when this task is first initialized.
	 */
	default void init() {

	}

	/**
	 * The hook called while this task is being executed. In the event
	 * that this task finishes immediately after init, this method is
	 * skipped.
	 */
	default void execute() {

	}

	/**
	 * Determines whether this task is finished. This is called after
	 * every {@link Task#init()} and {@link Task#execute()} invocation.
	 *
	 * @return {@code true} if this task should end.
	 */
	boolean isFinished();

	/**
	 * Handles the clean-up after this task finishes.
	 *
	 * @param interrupted Whether this task completed because it finished
	 *					or because the scheduler cancelled it.
	 */
	default void end(boolean interrupted) {
		
	}

	/**
	 * Gets a set of this task's lock keys. Lock keys can be any object,
	 * and are resources that if the task were to write to while another
	 * was using it, undefined behavior or errors would occur. Locks are
	 * assigned a priority value when they're scheduled based on factors
	 * such as whether they're tied to a button or are default commands.
	 *
	 * <p>In the event that two tasks are fighting for the same key, the
	 * task with the higher priority is allowed to run and the other is
	 * placed in a suspended (<b>NOT</b> ended) state where they can be
	 * resumed as soon as the higher task is finished.
	 *
	 * @return A non-null list of {@link Object}s to use as lock keys.
	 */
	default @NotNull Set<? extends @NotNull Object> getLockKeys() {
		return Set.of();
	}
}