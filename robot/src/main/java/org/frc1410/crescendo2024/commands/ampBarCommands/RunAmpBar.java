package org.frc1410.crescendo2024.commands.ampBarCommands;

import org.frc1410.crescendo2024.subsystems.AmpBar;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.crescendo2024.subsystems.LEDs;

import static org.frc1410.crescendo2024.util.Constants.*;

public class RunAmpBar extends Command {

	private final AmpBar ampBar;

	private boolean isExtended;

	public RunAmpBar(AmpBar ampBar, boolean isExtended) {
		this.ampBar = ampBar;
		this.isExtended = isExtended;
		addRequirements(this.ampBar);
	}

	@Override
	public void initialize() {
		if(!isExtended) {
			ampBar.setSpeed(AMP_BAR_SPEED_REVERSED);
		} else {
			ampBar.setSpeed(AMP_BAR_SPEED);
		}
	}

	@Override
	public void execute() {

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
