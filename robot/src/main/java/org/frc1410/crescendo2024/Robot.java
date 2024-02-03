package org.frc1410.crescendo2024;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import org.frc1410.crescendo2024.commands.ScoreAmp;
import org.frc1410.crescendo2024.commands.IncrementShooterRPM;
import org.frc1410.crescendo2024.commands.ShooterManual;
import org.frc1410.crescendo2024.util.NetworkTables;
import org.frc1410.crescendo2024.subsystems.AmpBar;
import org.frc1410.crescendo2024.subsystems.Intake;
import org.frc1410.crescendo2024.subsystems.Shooter;
import org.frc1410.crescendo2024.subsystems.Storage;
import org.frc1410.crescendo2024.commands.RunIntakeLooped;
import org.frc1410.framework.PhaseDrivenRobot;
import org.frc1410.framework.control.Controller;
import org.frc1410.framework.scheduler.task.TaskPersistence;

import static org.frc1410.crescendo2024.util.IDs.*;
import static org.frc1410.crescendo2024.util.Constants.*;

public final class Robot extends PhaseDrivenRobot {


	private final Controller driverController = new Controller(scheduler, DRIVER_CONTROLLER, 0.1 );
	private final Controller operatorController = new Controller(scheduler, OPERATOR_CONTROLLER,  0.1);
	private final Shooter shooter = subsystems.track(new Shooter());
	private final AmpBar ampBar = new AmpBar();
	private final Storage storage = new Storage();
	private final Intake intake = new Intake();

	private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
	private final NetworkTable table = nt.getTable("Auto");

	@Override
	public void autonomousSequence() {}

	@Override
	public void teleopSequence() {
		driverController.LEFT_TRIGGER.button().whileHeld(new ScoreAmp(shooter, storage), TaskPersistence.GAMEPLAY);

		// TODO: switch command to be on the driver controller.
		operatorController.RIGHT_BUMPER.whileHeld(new ShooterManual(shooter), TaskPersistence.GAMEPLAY);

		operatorController.A.whenPressed(new IncrementShooterRPM(shooter, SHOOTER_RPM_INCREMENT), TaskPersistence.GAMEPLAY);
		operatorController.B.whenPressed(new IncrementShooterRPM(shooter, -SHOOTER_RPM_INCREMENT), TaskPersistence.GAMEPLAY);

		operatorController.RIGHT_TRIGGER.button().whileHeld(new RunIntakeLooped(intake, storage, INTAKE_SPEED, STORAGE_SPEED), TaskPersistence.GAMEPLAY);
		operatorController.LEFT_TRIGGER.button().whileHeld(new RunIntakeLooped(intake, storage, OUTTAKE_SPEED, STORAGE_OUTTAKE_SPEED), TaskPersistence.GAMEPLAY);

	}

	@Override
	public void testSequence() {

	}

	@Override
	protected void disabledSequence() {

	}
}
