package org.frc1410.framework.scheduler.task;

import org.frc1410.framework.phase.Phase;

/**
 * A model for how long a task should survive. Tasks are checked for
 * collection whenever the robot changes state. Task persistence is
 * ignored when the robot enters {@link Phase#EMERGENCY_STOPPED}.
 *
 * @see Phase
 * @see TaskScheduler
 */
public enum TaskPersistence {

	/**
	 * Represents the ephemeral task persistence. Tasks with this
	 * priority are canceled upon phase transition into any phase
	 * including gameplay phases. Ephemeral tasks are good to use
	 * for single-phase jobs such as auto commands or subscribing
	 * to buttons.
	 */
	EPHEMERAL {

		@Override
		public boolean shouldPersist(Phase into) {
			return false;
		}
	},

	/**
	 * Represents the gameplay task persistence. Tasks with this priority
	 * are canceled upon transition into non-gameplay phases. The phases
	 * that tasks marked with this persistence will survive through will
	 * return {@code true} in {@link Phase#isGameplay()}. The only phases
	 * that do this are {@link Phase#AUTONOMOUS} and {@link Phase#TELEOP}.
	 *
	 * @see Phase#isGameplay()
	 */
	GAMEPLAY {

		@Override
		public boolean shouldPersist(Phase into) {
			return into.isGameplay();
		}
	},

	/**
	 * Represents the durable task persistence. Tasks with this priority
	 * are not canceled upon phase transition, even into disabled. It is
	 * important to note that tasks are <b>not</b> ticked when the robot
	 * is disabled, but rather they are held in a suspended state. Since
	 * they are not canceled, the {@link Task#end(boolean)} method does
	 * not get called. The task will simply resume execution after the
	 * phase transitions out of {@link Phase#DISABLED}.
	 *
	 * @implNote All tasks, even durable tasks, are canceled when the
	 *		   {@link Phase#EMERGENCY_STOPPED} is entered.
	 */
	DURABLE {

		@Override
		public boolean shouldPersist(Phase into) {
			return true;
		}
	};

	/**
	 * Checks if this task should survive when a phase
	 * transition completes.
	 *
	 * @param into The phase that was just entered
	 *
	 * @return {@code true} if the task should not be interrupted.
	 */
	public abstract boolean shouldPersist(Phase into);
}