package org.frc1410.framework.control;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import org.frc1410.framework.scheduler.task.*;
import org.frc1410.framework.scheduler.task.lock.LockPriority;

import static edu.wpi.first.wpilibj.XboxController.Axis.*;
import static edu.wpi.first.wpilibj.XboxController.Button.*;


public class Controller {

	final TaskScheduler scheduler;
	final XboxController backingController;

	public final Button A = new StandardButton(this, kA.value);
	public final Button B = new StandardButton(this, kB.value);
	public final Button X = new StandardButton(this, kX.value);
	public final Button Y = new StandardButton(this, kY.value);

	public final Button BACK = new StandardButton(this, kBack.value);
	public final Button START = new StandardButton(this, kStart.value);

	public final Button LEFT_BUMPER = new StandardButton(this, kLeftBumper.value);
	public final Button RIGHT_BUMPER = new StandardButton(this, kRightBumper.value);

	public final Button LEFT_STICK = new StandardButton(this, kLeftStick.value);
	public final Button RIGHT_STICK = new StandardButton(this, kRightStick.value);

	public final Axis LEFT_X_AXIS = new Axis(this, kLeftX.value);
	public final Axis RIGHT_X_AXIS = new Axis(this, kRightX.value);
	public final Axis LEFT_Y_AXIS = new Axis(this, kLeftY.value);
	public final Axis RIGHT_Y_AXIS = new Axis(this, kRightY.value);

	public final Axis LEFT_TRIGGER = new Axis(this, kLeftTrigger.value);
	public final Axis RIGHT_TRIGGER = new Axis(this, kRightTrigger.value);

	public final DPadButton DPAD_UP = new DPadButton(this, DPadButton.Direction.UP);
    public final DPadButton DPAD_UP_RIGHT = new DPadButton(this, DPadButton.Direction.UP_RIGHT);
    public final DPadButton DPAD_UP_LEFT = new DPadButton(this, DPadButton.Direction.UP_LEFT);
    public final DPadButton DPAD_LEFT = new DPadButton(this, DPadButton.Direction.LEFT);
    public final DPadButton DPAD_RIGHT = new DPadButton(this, DPadButton.Direction.RIGHT);
    public final DPadButton DPAD_DOWN = new DPadButton(this, DPadButton.Direction.DOWN);
    public final DPadButton DPAD_DOWN_LEFT = new DPadButton(this, DPadButton.Direction.DOWN_LEFT);
    public final DPadButton DPAD_DOWN_RIGHT = new DPadButton(this, DPadButton.Direction.DOWN_RIGHT);

	private int rumbleDepth = 0;
	final double deadzone;

	public Controller(TaskScheduler scheduler, int port, double deadzone) {
		this.scheduler = scheduler;
		this.backingController = new XboxController(port);
		this.deadzone = deadzone;
	}

	private void setRumble(boolean rumbling) {
		backingController.setRumble(GenericHID.RumbleType.kBothRumble, rumbling ? 1 : 0);
	}

	public void rumble(long durationMillis) {
		var timeout = System.currentTimeMillis() + durationMillis;
		scheduler.schedule(new RumbleTask(this, timeout), TaskPersistence.GAMEPLAY, Observer.NO_OP, LockPriority.NULL);

		setRumble(true);
	}

	private void popRumble() {
		rumbleDepth--;

		// should never be less than 0 but safety
		if (rumbleDepth <= 0) {
			rumbleDepth = 0;
			setRumble(false);
		}
	}

	private record RumbleTask(Controller controller, long timeout) implements Task {

		@Override
		public boolean isFinished() {
			return System.currentTimeMillis() > timeout;
		}

		@Override
		public void end(boolean interrupted) {
			controller.popRumble();
		}
	}
}