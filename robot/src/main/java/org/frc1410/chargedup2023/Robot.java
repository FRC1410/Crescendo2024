package org.frc1410.chargedup2023;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import org.frc1410.chargedup2023.util.NetworkTables;
import org.frc1410.framework.PhaseDrivenRobot;

public final class Robot extends PhaseDrivenRobot {

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
	public void teleopSequence() {}

	@Override
	public void testSequence() {

	}

	@Override
	protected void disabledSequence() {

	}
}
