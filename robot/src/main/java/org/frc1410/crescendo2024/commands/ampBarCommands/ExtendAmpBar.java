package org.frc1410.crescendo2024.commands.ampBarCommands;

import org.frc1410.crescendo2024.subsystems.AmpBar;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ExtendAmpBar extends ParallelRaceGroup {
	public ExtendAmpBar(AmpBar ampBar, double speed, double waitTime) {
		addCommands(
			new ParallelRaceGroup(
				new RunAmpBar(ampBar, speed),
				new WaitCommand(waitTime)
			)
		);
	}
}
