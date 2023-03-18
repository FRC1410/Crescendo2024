package org.frc1410.framework.scheduler.task;

/**
 * Models the current state of a task. States fall into three general categories:
 * <ul>
 *  <li><b>executing</b> – states for the normal execution of a task.</li>
 *  <li><b>inactive</b> – states indicating a task that is not currently running.</li>
 *  <li><b>terminated</b> – states indicating a task that has been completely stopped and will be unqueued.</li>
 * </ul>
 *
 * <p>Generally, the states in order are:
 * <ol>
 *  <li>{@link TaskState#FLAGGED_EXECUTION}</li>
 *  <li>{@link TaskState#EXECUTING}</li>
 *  <li>{@link TaskState#FLAGGED_COMPLETION}</li>
 *  <li>{@link TaskState#SUSPENDED}</li>
 * </ol>
 **/
public enum TaskState {

	// H1: executing

	/**
	 * Indicates that a task has been flagged for execution but has not been ticked
	 * yet. When the scheduler ticks a task in this state, its {@link Task#init()}
	 * hook is called, and depending on {@link Task#isFinished()}, it will either
	 * enter {@link TaskState#EXECUTING} or {@link TaskState#FLAGGED_COMPLETION}.
	 *
	 * <p>Category: <pre>executing</pre>
	 */
	FLAGGED_EXECUTION,

	/**
	 * Indicates that a task is currently executing and has completed its init
	 * cycle. When the scheduler ticks a task in this state, it calls the task
	 * {@link Task#execute()} hook. Then, if {@link Task#isFinished()} returns
	 * {@code true}, it will enter {@link TaskState#FLAGGED_COMPLETION}.
	 * is finished via {@link Task#isFinished()}, and if it is, enters
	 *
	 * <p>Category: <pre>executing</pre>
	 */
	EXECUTING,

	// H1: inactive

	/**
	 * Indicates that a task has come to an end naturally and is ready to be
	 * suspended and for its {@link Task#end(boolean)} hook to be called.
	 *
	 * <p>Category: <pre>inactive</pre>
	 */
	FLAGGED_COMPLETION,

	/**
	 * Indicates that a task should be interruped temporarily, generally by
	 * an {@link Observer}. For example, an observer can choose to suspend
	 * a task when a button is released, and it will interrupt the task.
	 *
	 * <br>This is the same as {@link TaskState#FLAGGED_COMPLETION} except
	 * it passed {@code true} for {@code interrupted}.
	 *
	 * <p>Category: <pre>inactive</pre>
	 */
	FLAGGED_SUSPENSION,

	/**
	 * Indicates that a task is suspended and should not be processed. The
	 * scheduler will skip any task in this state, but it will remain part
	 * of the loop's task registry.
	 *
	 * <p>Category: <pre>inactive</pre>
	 */
	SUSPENDED,

	// H1: inactive + terminated

	/**
	 * Entered when a task has been flagged for complete removal from
	 * its loop. The only case where this is currently used is when a
	 * phase transition occurs and a task has exceeded its lifetime.
	 *
	 * <p>Category: <pre>terminated</pre>
	 */
	FLAGGED_TERMINATION,

	/**
	 * A flag state used to hint to the loop that it should remove
	 * a task from its list. When a terminated task is ticked, it's
	 * immediately removed and ended.
	 *
	 * <p>Category: <pre>terminated</pre>
	 */
	TERMINATED;

	public boolean isExecuting() {
		return this == EXECUTING || this == FLAGGED_EXECUTION;
	}

	public boolean isInactive() {
		return !isExecuting();
	}

	public boolean isTerminated() {
		return this == FLAGGED_TERMINATION || this == TERMINATED;
	}
}