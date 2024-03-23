package org.frc1410.crescendo2024.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static org.frc1410.crescendo2024.util.Constants.SHOOTING_TIME;

import org.frc1410.crescendo2024.commands.RunStorage;
import org.frc1410.crescendo2024.subsystems.Drivetrain;
import org.frc1410.crescendo2024.subsystems.Intake;
import org.frc1410.crescendo2024.subsystems.LEDs;
import org.frc1410.crescendo2024.subsystems.Shooter;
import org.frc1410.crescendo2024.subsystems.Storage;

public class ShootNote extends ParallelRaceGroup {
	public ShootNote(Drivetrain drivetrain, Shooter shooter, Storage storage, Intake intake, LEDs leds, double shooterRPM, double storageRPM) {
		addCommands(
			new RunShooter(shooter, shooterRPM),

			new SequentialCommandGroup(
				new WaitCommand(0.8),

				new ParallelRaceGroup(
					new WaitCommand(SHOOTING_TIME),
					new RunStorage(storage, storageRPM)
				)
			)
		);
	}
}
