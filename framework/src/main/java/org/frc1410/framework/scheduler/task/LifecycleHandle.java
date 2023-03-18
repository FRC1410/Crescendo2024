package org.frc1410.framework.scheduler.task;

import org.jetbrains.annotations.ApiStatus;
import org.jetbrains.annotations.NotNull;

/**
 * This class is responsible for external management of a task's
 * lifecycle. It contains a state, and allows other objects such
 * as {@link Observer}s to manage it.
 */
public final class LifecycleHandle {

	public @NotNull TaskState state = TaskState.SUSPENDED;

	/**
	 * Requests that the command start executing. This is the
	 * resting state for a task.
	 */
	public void requestExecution() {
		if (!state.isExecuting()) {
			state = TaskState.FLAGGED_EXECUTION;
		}
	}

	/**
	 * Requests that a command enter a suspended state where it is not
	 * ticked but can still be resumved by its obserer. Suspension will
	 * call the task's {@link Task#end(boolean)} hook, interrupting it.
	 */
	public void requestSuspension() {
		if (!state.isInactive()) {
			state = TaskState.FLAGGED_SUSPENSION;
		}
	}

	/**
	 * Requests that a command be terminated and dropped entirely from its
	 * loop. When a task is terminated, its {@link Task#end(boolean)} hook
	 * is called, interrupting it.
	 *
	 * @apiNote It is not recommended that this be used outside of framework
	 *		  internals, as most of the time suspension is wanted. Marking
	 *		  a task as terminated makes it completely inaccessible in the
	 *		  future, so it cannot be resumed.
	 */
	@ApiStatus.Internal
	public void requestTermination() {
		if (!state.isTerminated()) {
			state = TaskState.FLAGGED_TERMINATION;
		}
	}

	@Override
	public String toString() {
		return "LifecycleHandle[state=" + state + "]";
	}
}