package org.frc1410.crescendo2024.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frc1410.crescendo2024.commands.RunStorage;
import org.frc1410.crescendo2024.subsystems.Shooter;
import org.frc1410.crescendo2024.subsystems.Storage;

public class AutomaticShoot extends ParallelRaceGroup {

	public AutomaticShoot(Shooter shooter, Storage storage, double desiredShooterRPM, double desiredStorageRPM) {
		addCommands(
			new RunShooterLooped(shooter, desiredShooterRPM),

			new SequentialCommandGroup(
				new WaitCommand(1),

				new ParallelRaceGroup(
					new WaitCommand(0.5),
					new RunStorage(storage, desiredStorageRPM)
				)
			)
		);
	}
}
