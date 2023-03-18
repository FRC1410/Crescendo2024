package org.frc1410.framework.control.observer;

import org.frc1410.framework.control.Button;
import org.frc1410.framework.scheduler.task.LifecycleHandle;
import org.frc1410.framework.scheduler.task.Observer;
import org.jetbrains.annotations.NotNull;

public class WhenPressedObserver implements Observer {

	private final Button button;
	private boolean wasActive;

	public WhenPressedObserver(Button button) {
		this.button = button;
	}

	@Override
	public void tick(@NotNull LifecycleHandle handle) {
		if (!wasActive && button.isActive()) {
			handle.requestExecution();
			wasActive = true;
		}

		wasActive = button.isActive();
	}
}