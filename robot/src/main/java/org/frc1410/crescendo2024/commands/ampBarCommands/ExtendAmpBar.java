package org.frc1410.crescendo2024.commands.ampBarCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc1410.crescendo2024.subsystems.AmpBar;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frc1410.crescendo2024.subsystems.LEDs;

import static org.frc1410.crescendo2024.util.Constants.*;

public class ExtendAmpBar extends SequentialCommandGroup {

	public ExtendAmpBar(AmpBar ampBar, LEDs leds, boolean isExtended) {

		addCommands(
			new ParallelRaceGroup(
				new RunAmpBar(ampBar, isExtended),
				new WaitCommand(AMP_BAR_TIMER)
			)
			// new InstantCommand(() -> {
			// 	ampBar.isExtended = isExtended;
			// 	// if(isExtended) {
			// 	// 	leds.changeLEDsColor(LEDs.Colors.AMP_ARM_FIRE_ANIMATION);
			// 	// } else {
			// 	// 	leds.defaultLEDsState();
			// 	// }
			// })
		);

	}
}
