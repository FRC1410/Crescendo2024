package org.frc1410.crescendo2024;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.frc1410.crescendo2024.commands.*;
import org.frc1410.crescendo2024.commands.ampBarCommands.ScoreAmp;
import org.frc1410.crescendo2024.commands.drivetrainCommands.DriveLooped;
import org.frc1410.crescendo2024.commands.shooterCommands.IncrementShooterRPM;
import org.frc1410.crescendo2024.commands.shooterCommands.ShooterManual;
import org.frc1410.crescendo2024.subsystems.*;
import org.frc1410.crescendo2024.util.NetworkTables;
import org.frc1410.framework.AutoSelector;

import org.frc1410.framework.PhaseDrivenRobot;
import org.frc1410.framework.control.Controller;
import org.frc1410.framework.scheduler.task.TaskPersistence;

import static org.frc1410.crescendo2024.util.IDs.*;
import static org.frc1410.crescendo2024.util.Constants.*;

public final class Robot extends PhaseDrivenRobot {

	public Robot() {
		NamedCommands.registerCommand("ShooterManual", new ShooterManual(shooter));

		NamedCommands.registerCommand("RunIntakeLooped", new RunIntakeLooped(
			intake,
			storage,
			1000,
			1000
		));
	}

	private final Controller driverController = new Controller(scheduler, DRIVER_CONTROLLER, 0.1 );
	private final Controller operatorController = new Controller(scheduler, OPERATOR_CONTROLLER,  0.1);
	private final Drivetrain drivetrain = subsystems.track(new Drivetrain(subsystems));
	private final Shooter shooter = subsystems.track(new Shooter());
	private final AmpBar ampBar = new AmpBar();
	private final Storage storage = subsystems.track(new Storage());
	private final Intake intake = new Intake();

	private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
	private final NetworkTable table = nt.getTable("Auto");

	private final AutoSelector autoSelector = new AutoSelector()
		.add("New Auto", () -> new PathPlannerAuto("New Auto"))
		.add("Test", () -> new PathPlannerAuto("Test"));

	{
		var profiles = new String[autoSelector.getProfiles().size()];
		for (var i = 0; i < profiles.length; i++) {
			profiles[i] = autoSelector.getProfiles().get(i).name();
		}

		var pub = NetworkTables.PublisherFactory(table, "Choices", profiles);
		pub.accept(profiles);
	}


	private final StringPublisher autoPublisher = NetworkTables.PublisherFactory(table, "Profile",
		autoSelector.getProfiles().isEmpty() ? "" : autoSelector.getProfiles().get(0).name());

	private final StringSubscriber autoSubscriber = NetworkTables.SubscriberFactory(table, autoPublisher.getTopic());

	@Override
	public void autonomousSequence() {

		NetworkTables.SetPersistence(autoPublisher.getTopic(), true);
		String autoProfile = autoSubscriber.get();
		var autoCommand = autoSelector.select(autoProfile);

		scheduler.scheduleAutoCommand(autoCommand);
	}

	@Override
	public void teleopSequence() {

		scheduler.scheduleDefaultCommand(new DriveLooped(drivetrain, driverController.LEFT_Y_AXIS, driverController.LEFT_X_AXIS, driverController.RIGHT_X_AXIS), TaskPersistence.EPHEMERAL);

		driverController.Y.whenPressed(new InstantCommand(
			() -> drivetrain.zeroYaw()
		), TaskPersistence.GAMEPLAY);

		driverController.LEFT_TRIGGER.button().whileHeld(new ScoreAmp(shooter, storage, false), TaskPersistence.GAMEPLAY);

		// TODO: switch command to be on the driver controller.
		operatorController.RIGHT_BUMPER.whileHeld(new ShooterManual(shooter), TaskPersistence.GAMEPLAY);

		operatorController.A.whenPressed(new IncrementShooterRPM(shooter, SHOOTER_RPM_INCREMENT), TaskPersistence.GAMEPLAY);
		operatorController.B.whenPressed(new IncrementShooterRPM(shooter, -SHOOTER_RPM_INCREMENT), TaskPersistence.GAMEPLAY);

		operatorController.RIGHT_TRIGGER.button().whileHeld(new RunIntakeLooped(intake, storage, INTAKE_SPEED, -1), TaskPersistence.GAMEPLAY);
		operatorController.LEFT_TRIGGER.button().whileHeld(new RunIntakeLooped(intake, storage, OUTTAKE_SPEED, STORAGE_OUTTAKE_SPEED), TaskPersistence.GAMEPLAY);

	}

	@Override
	public void testSequence() {

	}

	@Override
	protected void disabledSequence() {

	}
}
