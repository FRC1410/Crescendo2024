package org.frc1410.chargedup2023;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.frc1410.chargedup2023.commands.RunIntakeLooped;
import org.frc1410.chargedup2023.commands.RunShooterLooped;
import org.frc1410.chargedup2023.commands.RunStorage;
import org.frc1410.chargedup2023.subsystems.Intake;
import org.frc1410.chargedup2023.subsystems.Shooter;
import org.frc1410.chargedup2023.subsystems.Storage;
import org.frc1410.chargedup2023.util.NetworkTables;
import org.frc1410.framework.PhaseDrivenRobot;
import org.frc1410.framework.control.Controller;
import org.frc1410.framework.scheduler.task.TaskPersistence;

import static org.frc1410.chargedup2023.util.Constants.*;

public final class Robot extends PhaseDrivenRobot {

	private final Controller operatorController = new Controller(scheduler, OPERATOR_CONTROLLER,  0.1);

	private final Shooter shooter = new Shooter();
	private final Storage storage = new Storage();
	private final Intake intake = new Intake();


	//<editor-fold desc="Controllers">
	//</editor-fold>

	//<editor-fold desc="Auto Selector">
	private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
	private final NetworkTable table = nt.getTable("Auto");

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

	//</editor-fold>

	@Override
	public void autonomousSequence() {}

	@Override
	public void teleopSequence() {

		scheduler.scheduleDefaultCommand(
			new RunShooterLooped(
				shooter,
				operatorController.RIGHT_TRIGGER
			),
			TaskPersistence.GAMEPLAY
		);

		operatorController.A.whileHeld(
			new RunStorage(
				storage,
				false
			), TaskPersistence.GAMEPLAY
		);

		operatorController.B.whileHeld(
			new RunStorage(
				storage,
				true
			), TaskPersistence.GAMEPLAY
		);

		operatorController.RIGHT_BUMPER.whileHeld(
			new RunIntakeLooped(
				intake,
				false
			), TaskPersistence.GAMEPLAY
		);

		operatorController.LEFT_BUMPER.whileHeld(
			new RunIntakeLooped(
				intake,
				true
			), TaskPersistence.GAMEPLAY
		);
	}

	@Override
	public void testSequence() {

	}

	@Override
	protected void disabledSequence() {

	}
}
