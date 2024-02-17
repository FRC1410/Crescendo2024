package org.frc1410.crescendo2024.commands.ampBarCommands;

import org.frc1410.crescendo2024.subsystems.AmpBar;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frc1410.crescendo2024.subsystems.LEDs;

public class ExtendAmpBar extends ParallelRaceGroup {
	private LEDs leds = new LEDs();



	public ExtendAmpBar(AmpBar ampBar, double speed, double waitTime) {

		addCommands(
			new ParallelRaceGroup(
				new RunAmpBar(ampBar, speed),
				new WaitCommand(waitTime)
			)
		);

	}
}
