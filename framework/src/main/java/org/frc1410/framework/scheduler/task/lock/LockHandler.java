package org.frc1410.framework.scheduler.task.lock;

import org.frc1410.framework.scheduler.task.BoundTask;
import org.frc1410.framework.scheduler.task.TaskScheduler;
import org.frc1410.framework.util.log.Logger;
import org.jetbrains.annotations.NotNull;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * This class is responsible for the storage and monitoring of locks. The
 * {@link TaskScheduler} owns an instance, and uses it each tick to check
 * if a task it wants to run owns all of its locks.
 */
public final class LockHandler {

	private static final Logger LOG = new Logger("LockHandler");
	private final Map<Object, @NotNull BoundTask> owners = new ConcurrentHashMap<>();

	/**
	 * Checks if the provided task owns access to all of its lock keys. If a
	 * task owns all of its keys, it is allowed to execute. If another task
	 * has prior claim over a key but has lower priority, ownership of that
	 * key is transferred to the other task. A task has to own all keys to
	 * take ownership of them.
	 *
	 * @param task The task to check.
	 *
	 * @return {@code true} if the task owns all of its locks and can execute.
	 */
	public boolean ownsLocks(@NotNull BoundTask task) {
		// Ignore cases where a task has no lock or is not running
		if (task.lock() == null || !task.handle().state.isExecuting()) return true;

		for (var key : task.lock().keys()) {
			var owner = owners.getOrDefault(key, task);
			if (owner == task || owner.lock() == null) {
				// Skip processing if this task owns the lock
				continue;
			}

			if (owner.lock().priority() > task.lock().priority()) {
				return false;
			}
		}


//		LOG.info("Task %s owns all of its locks, transferring ownership...", task);

		// Take ownership
		for (var key : task.lock().keys()) {
			owners.put(key, task);
		}

		return true;
	}

	/**
	 * Releases all locks the task currently holds.
	 *
	 * @param task The task to release.
	 */
	public void releaseLocks(@NotNull BoundTask task) {
		if (task.lock() == null) {
			return;
		}

		for (var key : task.lock().keys()) {
			owners.remove(key, task);
		}
	}
}