package org.frc1410.chargedup2023.commands.actions;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frc1410.chargedup2023.commands.RunAmpBar;
import org.frc1410.chargedup2023.subsystems.AmpBar;
import static org.frc1410.chargedup2023.util.Tuning.*;

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
