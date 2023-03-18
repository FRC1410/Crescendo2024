package org.frc1410.framework.control.observer;

import org.frc1410.framework.control.Button;
import org.frc1410.framework.scheduler.task.LifecycleHandle;
import org.frc1410.framework.scheduler.task.Observer;
import org.jetbrains.annotations.NotNull;

public class WhileHeldOnceObserver implements Observer {

	private final Button button;
	private boolean wasActive = false;

	public WhileHeldOnceObserver(Button button) {
		this.button = button;
	}

	@Override
	public void tick(@NotNull LifecycleHandle handle) {
		if (!wasActive && button.isActive()) {
			handle.requestExecution();
			wasActive = true;
		}

		if (!button.isActive()) {
			handle.requestSuspension();
		}

		wasActive = button.isActive();
	}
}