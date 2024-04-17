package org.frc1410.crescendo2024.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.RPM;
import static org.frc1410.crescendo2024.util.Constants.OUTTAKE_SPEED;
import static org.frc1410.crescendo2024.util.Constants.SHOOTER_OUTTAKE_VELOCITY;
import static org.frc1410.crescendo2024.util.Constants.STORAGE_OUTTAKE_VELOCITY;

import org.frc1410.crescendo2024.subsystems.Intake;
import org.frc1410.crescendo2024.subsystems.Shooter;
import org.frc1410.crescendo2024.subsystems.Storage;

public class OuttakeNote extends Command {
	private final Intake intake;
	private final Storage storage;
	private final Shooter shooter;

	public OuttakeNote(Intake intake, Storage storage, Shooter shooter) {
		this.intake = intake;
		this.storage = storage;
		this.shooter = shooter;

		this.addRequirements(this.intake);
	}

	@Override
	public void initialize() {
		this.intake.setSpeed(-OUTTAKE_SPEED);
		this.storage.setVelocity(STORAGE_OUTTAKE_VELOCITY.negate());
		this.shooter.setVelocity(SHOOTER_OUTTAKE_VELOCITY.negate());
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.intake.setSpeed(0);
		this.storage.setVelocity(RPM.zero());
		this.shooter.setVelocity(RPM.zero());
	}
}
