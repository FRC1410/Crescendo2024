package org.frc1410.crescendo2024.commands.ampBarCommands;

import org.frc1410.crescendo2024.subsystems.AmpBar;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.crescendo2024.subsystems.LEDs;


public class RunAmpBar extends Command {

	private final AmpBar ampBar;
	private final double ampSpeed;

	private boolean extended;

	LEDs leds = new LEDs();

	public RunAmpBar(AmpBar ampBar, double ampSpeed) {
		this.ampBar = ampBar;
		this.ampSpeed = ampSpeed;
		extended = true;
		addRequirements(this.ampBar);
	}

	@Override
	public void initialize() {
		ampBar.setSpeed(ampSpeed);
		extended = !extended;
	}

	@Override
	public void execute() {
		if(extended) {
			leds.changeLEDsColor(LEDs.Colors.AMP_ARM_FIRE_ANIMATION);
		} else {
			leds.defaultLEDsState();
		}
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
