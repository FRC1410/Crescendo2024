package org.frc1410.crescendo2024.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frc1410.crescendo2024.commands.Intake.RunIntake;
import org.frc1410.crescendo2024.commands.RunStorage;
import org.frc1410.crescendo2024.subsystems.Intake;
import org.frc1410.crescendo2024.subsystems.Shooter;
import org.frc1410.crescendo2024.subsystems.Storage;

public class Shoot extends ParallelRaceGroup {

	public Shoot(Shooter shooter, Storage storage, Intake intake, double desiredShooterRPM, double desiredStorageRPM) {
		addCommands(
			new RunShooterLooped(shooter, desiredShooterRPM),

			new SequentialCommandGroup(
				//Shooter length
				new WaitCommand(1.5),

				new ParallelRaceGroup(
					new WaitCommand(0.5),
					new RunStorage(storage, desiredStorageRPM),
					new RunIntake(intake, 0.5)
				)
			)
		);
	}
}
