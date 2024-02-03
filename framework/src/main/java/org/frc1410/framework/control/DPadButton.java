package org.frc1410.framework.control;

import org.frc1410.framework.scheduler.task.TaskScheduler;

public class DPadButton implements Button {

	private final Controller controller;
	private final Direction direction;

	public DPadButton(Controller controller, Direction direction) {
		this.controller = controller;
		this.direction = direction;
	}

	@Override
	public TaskScheduler scheduler() {
		return controller.scheduler;
	}

	@Override
	public boolean isActive() {
		return controller.backingController.getPOV() == direction.angle;
	}

    public enum Direction {
        UP,
        UP_RIGHT,
        RIGHT,
        DOWN_RIGHT,
        DOWN,
        DOWN_LEFT,
        LEFT,
        UP_LEFT,
        ;
          
        private final int angle = ordinal() * 45; // UP is 0, clockwise increments of 45
    }
}