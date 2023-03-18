package org.frc1410.framework;

import edu.wpi.first.wpilibj.TimedRobot;
import org.frc1410.framework.phase.Phase;
import org.frc1410.framework.phase.PhaseController;
import org.frc1410.framework.scheduler.loop.Loop;
import org.frc1410.framework.scheduler.subsystem.SubsystemStore;
import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;
import org.frc1410.framework.scheduler.task.TaskScheduler;
import org.frc1410.framework.util.log.Logger;

/**
 * The base robot type that all robots can extend to access the scheduling
 * and phase features. {@link PhaseDrivenRobot}s hold a state machine used
 * for the current phase, a task scheduler, and a subsystem store.
 *
 * <p>Robots are built by scheduling tasks to {@link Phase}s, where there
 * are hooks for each phase that can be used to add commands and buttons.
 *</p>
 *
 * <p>This class is also responsible for handling a robot's subsystems.
 * In order to get a subsystem with a {@link TickedSubsystem#periodic()}
 * handle to work, it must be added through the {@link SubsystemStore}.
 */
public abstract class PhaseDrivenRobot extends TimedRobot {

	private static final Logger LOG = new Logger("Robot");

	protected final TaskScheduler scheduler = new TaskScheduler();
	protected final PhaseController phaseController = new PhaseController(scheduler);
	protected final SubsystemStore subsystems = new SubsystemStore(scheduler);

	public PhaseDrivenRobot() {
		super();
	}

	public PhaseDrivenRobot(double period) {
		super(period);
	}

	@Override
	public final void robotPeriodic() {
		if (phaseController.isTransitioning()) {
			LOG.warn("Scheduler tick submitted during transition. Skipped.");
			return;
		}

		// Tick the main loop. This loop just runs on the default robot period.
		scheduler.loopStore.main.tick();

		{
			// Grab the queue of untracked loops.
			var loops = scheduler.loopStore.getUntrackedLoops();
			if (loops.isEmpty()) return; // Optimization: early return

			// This is a fast way to iterate over all untracked loops and to schedule them while popping them.
			// This is optimized to prevent tick overruns as it executes each tick on periodic.
			Loop loop;
			while ((loop = loops.pollFirst()) != null) {
				addPeriodic(loop::tick, loop.getPeriodSeconds());
			}
		}
	}

	// <editor-fold desc="> Phase hooks" defaultstate="collapsed">

	@Override
	public final void robotInit() {
		LOG.info("Robot initialized.");
		// Signal that we're about to transition out of INIT as soon as the scheduler does a sweep
		phaseController.beginTransition();
	}

	/**
	 * The hook for when a robot enters the {@link Phase#DISABLED} phase.
	 * It should be used to set up any tasks that should be executed in
	 * this phase.
	 */
	protected void disabledSequence() {

	}

	/**
	 * The hook for when a robot enters the {@link Phase#AUTONOMOUS} phase.
	 * It should be used to set up any tasks that should be executed in
	 * this phase.
	 */
	protected void autonomousSequence() {

	}

	/**
	 * The hook for when a robot enters the {@link Phase#TELEOP} phase.
	 * It should be used to set up any tasks that should be executed in
	 * this phase.
	 */
	protected void teleopSequence() {

	}

	/**
	 * The hook for when a robot enters the {@link Phase#TEST} phase.
	 * It should be used to set up any tasks that should be executed in
	 * this phase.
	 */
	protected void testSequence() {

	}

	// Initialization methods.
	@Override
	public final void disabledInit() {
		phaseController.transition(Phase.DISABLED);
		disabledSequence();
	}

	@Override
	public final void autonomousInit() {
		phaseController.transition(Phase.AUTONOMOUS);
		autonomousSequence();
	}

	@Override
	public final void teleopInit() {
		phaseController.transition(Phase.TELEOP);
		teleopSequence();
	}

	@Override
	public final void testInit() {
		phaseController.transition(Phase.TEST);
		testSequence();
	}

	@Override
	public final void disabledExit() {
		phaseController.beginTransition();
	}

	@Override
	public final void autonomousExit() {
		phaseController.beginTransition();
	}

	@Override
	public final void teleopExit() {
		phaseController.beginTransition();
	}

	@Override
	public final void testExit() {
		phaseController.beginTransition();
	}

	// These methods have a default implementation that prints a warning. This adds overhead to the scheduler
	// that we want to avoid, so we just have blank methods.

	@Override
	public final void simulationPeriodic() {

	}

	@Override
	public final void disabledPeriodic() {

	}

	@Override
	public final void autonomousPeriodic() {

	}

	@Override
	public final void teleopPeriodic() {

	}

	@Override
	public final void testPeriodic() {

	}
	// </editor-fold>
}
