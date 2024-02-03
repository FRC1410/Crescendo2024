package org.frc1410.crescendo2024.commands;

import org.frc1410.crescendo2024.subsystems.AmpBar;

import edu.wpi.first.wpilibj2.command.Command;


public class RunAmpBar extends Command {

	private final AmpBar ampBar;
	private final double ampSpeed;

	public RunAmpBar(AmpBar ampBar, double ampSpeed) {
		this.ampBar = ampBar;
		this.ampSpeed = ampSpeed;
		addRequirements(this.ampBar);
	}

	@Override
	public void initialize() {
		ampBar.setSpeed(ampSpeed);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		ampBar.setSpeed(0);
	}
}
