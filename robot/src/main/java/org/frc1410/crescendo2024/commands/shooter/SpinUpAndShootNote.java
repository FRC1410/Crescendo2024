package org.frc1410.crescendo2024.commands.shooter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.frc1410.crescendo2024.subsystems.Drivetrain;
import org.frc1410.crescendo2024.subsystems.Intake;
import org.frc1410.crescendo2024.subsystems.LEDs;
import org.frc1410.crescendo2024.subsystems.Shooter;
import org.frc1410.crescendo2024.subsystems.Storage;

public class SpinUpAndShootNote extends ParallelRaceGroup {
	public SpinUpAndShootNote(
		Drivetrain drivetrain, 
		Shooter shooter, 
		Storage storage, 
		Intake intake, 
		LEDs leds, 
		Measure<Velocity<Angle>> shooterVelocity, 
		Measure<Velocity<Angle>> storageVelocity
	) {
		addCommands(
			new RunShooter(shooter, shooterVelocity, true),

			new SequentialCommandGroup(
				new WaitForShooterVelocity(shooter, shooterVelocity),
				new ShootSpeaker(storage, intake)
			)
		);
	}
}
