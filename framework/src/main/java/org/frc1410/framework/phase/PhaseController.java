package org.frc1410.framework.phase;

import org.frc1410.framework.PhaseDrivenRobot;
import org.frc1410.framework.scheduler.task.TaskScheduler;
import org.frc1410.framework.util.log.Logger;

/**
 * A state machine responsible for controlling robot phase. This class is used in
 * the {@link PhaseDrivenRobot}, which handles the transition between phases when
 * mode enter/exit methods are called.
 */
public class PhaseController {

	private static final Logger LOG = new Logger("PhaseController");

	private final TaskScheduler scheduler;
	private Phase phase = Phase.INIT;
	private Phase oldPhase = null;

	public PhaseController(TaskScheduler scheduler) {
		this.scheduler = scheduler;
	}

	public void beginTransition() {
		LOG.debug("[PhaseController] Transitioning out of %s...", phase);

		oldPhase = phase;
		phase = Phase.TRANSITION;
	}

	public void transition(Phase phase) {
		if (oldPhase == null) {
			throw new IllegalStateException("Transition request was not submitted! This will lead to race conditions.");
		}

		LOG.info("Transition complete: %s -> %s", oldPhase, phase);

		this.oldPhase = null;
		this.phase = phase;

		scheduler.loopStore.propagateTransition(this.phase);
		scheduler.printState();
	}

	public Phase getPhase() {
		if (isTransitioning()) {
			throw new IllegalStateException("Cannot acquire phase during transition.");
		}

		return phase;
	}

	public boolean isTransitioning() {
		return phase == Phase.TRANSITION;
	}
}
