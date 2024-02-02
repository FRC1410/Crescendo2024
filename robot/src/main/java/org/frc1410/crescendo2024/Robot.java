package org.frc1410.crescendo2024;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.frc1410.framework.PhaseDrivenRobot;

import static org.frc1410.crescendo2024.util.Constants.*;

public final class Robot extends PhaseDrivenRobot {

	private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
	private final NetworkTable table = nt.getTable("Auto");

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
