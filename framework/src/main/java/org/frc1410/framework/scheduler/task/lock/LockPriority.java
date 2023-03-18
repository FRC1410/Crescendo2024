package org.frc1410.framework.scheduler.task.lock;

public sealed interface LockPriority permits NoInherit {

	int NULL = -1;

	int LOWEST = 0;
	int LOW = 1;
	int NORMAL = 2;
	int HIGH = 3;
	int HIGHEST = 4;

	static void requireInRange(int lockPriority) {
		if (lockPriority < -1 || lockPriority > 4) {
			throw new IllegalArgumentException("Lock priority must be one of: NULL (-1), LOWEST (0), LOW (1), NORMAL (2), HIGH (3), HIGHEST (4)");
		}
	}
}

final class NoInherit implements LockPriority {}