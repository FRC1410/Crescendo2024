package org.frc1410.crescendo2024;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.frc1410.framework.PhaseDrivenRobot;
import org.frc1410.framework.control.Controller;
import org.frc1410.framework.scheduler.task.TaskPersistence;

import static org.frc1410.crescendo2024.util.Constants.*;

import org.frc1410.crescendo2024.Commands.DriveLooped;
import org.frc1410.crescendo2024.Subsystems.Drivetrain;
import org.frc1410.crescendo2024.Subsystems.LEDs;

public final class Robot extends PhaseDrivenRobot {

	private final Controller driverController = new Controller(scheduler, 0, 0.2);

	private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
	private final NetworkTable table = nt.getTable("Auto");

	private final Drivetrain drivetrain = subsystems.track(new Drivetrain(subsystems));

	private final LEDs leds = new LEDs();

	@Override
	public void autonomousSequence() {}

	@Override
	public void teleopSequence() {
		scheduler.scheduleDefaultCommand(new DriveLooped(drivetrain, driverController.LEFT_Y_AXIS, driverController.LEFT_X_AXIS, driverController.RIGHT_X_AXIS), TaskPersistence.EPHEMERAL);
		driverController.B.whenPressed(
			new InstantCommand(
				() -> { drivetrain.zeroYaw(); }
			),
			TaskPersistence.EPHEMERAL
		);
	}

	@Override
	public void testSequence() {

	}

	@Override
	protected void disabledSequence() {

	}
}
