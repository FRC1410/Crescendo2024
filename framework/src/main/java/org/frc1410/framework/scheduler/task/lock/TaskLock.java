package org.frc1410.framework.scheduler.task.lock;

import org.intellij.lang.annotations.MagicConstant;
import org.jetbrains.annotations.NotNull;

import java.util.Set;

public record TaskLock(@MagicConstant(valuesFromClass = LockPriority.class) int priority, @NotNull Set<?> keys) {

	public TaskLock(@MagicConstant(valuesFromClass = LockPriority.class) int priority, Set<?> keys) {
		this.priority = priority;
		this.keys = keys;
	}
}