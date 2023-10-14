package org.frc1410.chargedup2023;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import org.frc1410.chargedup2023.util.NetworkTables;
import org.frc1410.framework.AutoSelector;
import org.frc1410.framework.PhaseDrivenRobot;
import org.frc1410.framework.control.Controller;
import org.frc1410.framework.scheduler.task.TaskPersistence;

import static org.frc1410.chargedup2023.util.Constants.*;

import org.frc1410.chargedup2023.Commands.DriveLooped;
import org.frc1410.chargedup2023.Commands.LockDrivetrainHeld;
import org.frc1410.chargedup2023.Commands.Auto.Engage;
import org.frc1410.chargedup2023.Commands.Auto.Forward;
import org.frc1410.chargedup2023.Subsystems.Drivetrain;

public final class Robot extends PhaseDrivenRobot {

	//<editor-fold desc="Controllers">
	private final Controller driverController = new Controller(scheduler, DRIVER_CONTROLLER, 0.2);
	private final Controller operatorController = new Controller(scheduler, OPERATOR_CONTROLLER, 0.2);
	//</editor-fold>

	//<editor-fold desc="Auto Selector">
	private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
	private final NetworkTable table = nt.getTable("Auto");

	private final Drivetrain drivetrain = subsystems.track(new Drivetrain(subsystems));

	{
		var layout = """
		[{
			"tabName": "Drive",
			"id": "drive",

			"components": [{
				"type": "string_select",
				"title": "Auto Selection",
				"layout": {
					"pos": [1, 1],
					"size": [2, 1]
				},
				"topics": ["Auto/Choices", "Auto/Selection"]
			}, {
				"type": "clock",
				"title": "Game Time",
				"layout": {
					"pos": [3, 1],
					"size": [2, 1]
				},
				"topics": ["FMSInfo/GameTime"]
			}, {
				"type": "node_select",
				"title": "Selected Node",
				"layout": {
					"pos": [5, 1],
					"size": [1, 1]
				},
				"topics": ["Drivetrain/Scoring Pose Index"]
			}, {
				"type": "boolean",
				"title": "LBork Line Break",
				"layout": {
					"pos": [6, 1],
					"size": [1, 1]
				},
				"topics": ["LBork/Line Break"]
			}]
		}]""";

		// grid, line break, auto, time
		var pub = NetworkTables.PublisherFactory(nt.getTable("viridian"), "layout", layout);
	}

	private final AutoSelector autoSelector = new AutoSelector()
	.add("ENGAGE", () -> new Engage(this.drivetrain))
		.add("FORWARD", () -> new Forward(this.drivetrain));

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
	//</editor-fold>

	@Override
	public void autonomousSequence() {
		NetworkTables.SetPersistence(autoPublisher.getTopic(), true);
		String autoProfile = autoSubscriber.get();
		var autoCommand = autoSelector.select(autoProfile);
		scheduler.scheduleAutoCommand(autoCommand);
	}

	@Override
	public void teleopSequence() {
		scheduler.scheduleDefaultCommand(new DriveLooped(drivetrain, driverController.LEFT_Y_AXIS, driverController.LEFT_X_AXIS, driverController.RIGHT_X_AXIS), TaskPersistence.GAMEPLAY);
		driverController.A.whileHeld(new LockDrivetrainHeld(drivetrain), TaskPersistence.EPHEMERAL);
		driverController.B.whenPressed(
			new InstantCommand(
				() -> { drivetrain.zeroYaw(); }
			),
			TaskPersistence.EPHEMERAL
		);
		// drivetrain.zero();
	}

	@Override
	public void testSequence() {

	}

	@Override
	protected void disabledSequence() {

	}
}
