package org.frc1410.crescendo2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.crescendo2024.subsystems.Climber;
import org.frc1410.framework.control.Axis;

public class ClimbLooped extends Command {
	private final Climber climber;

	private final Axis leftYAxis;
	private final Axis rightYAxis;

	public ClimbLooped(Climber climber, Axis leftYAxis, Axis rightYAxis) {
		this.climber = climber;

		this.leftYAxis = leftYAxis;
		this.rightYAxis = rightYAxis;

		this.addRequirements(climber);
	}

	@Override
	public void execute() {
		this.climber.setLeftClimberSpeed(this.leftYAxis.get());
		this.climber.setRightClimberSpeed(this.rightYAxis.get());
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.climber.setLeftClimberSpeed(0);
		this.climber.setRightClimberSpeed(0);
	}
}