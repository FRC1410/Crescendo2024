package org.frc1410.crescendo2024.commands.Intake;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frc1410.crescendo2024.commands.RunIntakeLooped;
import org.frc1410.crescendo2024.subsystems.Intake;
import org.frc1410.crescendo2024.subsystems.Storage;

public class IntakeLimit extends SequentialCommandGroup {

	public IntakeLimit(Intake intake, Storage storage, double intakeSpeed, double storageRPM) {
		addCommands(
			new RunIntakeLooped(intake, storage, intakeSpeed, storageRPM),

			new ParallelRaceGroup(

			)
		);
	}
}
