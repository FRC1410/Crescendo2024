package org.frc1410.crescendo2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.crescendo2024.subsystems.Climb;
import org.frc1410.framework.control.Axis;

public class ClimbLooped extends Command {

	private final Climb climb;

	private final Axis leftYAxis;
	private final Axis rightYAxis;

	public ClimbLooped(Climb climb, Axis leftYAxis, Axis rightYAxis) {
		this.climb = climb;

		this.leftYAxis = leftYAxis;
		this.rightYAxis = rightYAxis;
		addRequirements(climb);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		climb.setLeftClimberSpeed(leftYAxis.get());
		climb.setRightClimberSpeed(rightYAxis.get());
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		climb.setLeftClimberSpeed(0);
		climb.setRightClimberSpeed(0);
	}
}