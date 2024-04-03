package org.frc1410.crescendo2024.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;

import static org.frc1410.crescendo2024.util.Constants.OUTTAKE_SPEED;
import static org.frc1410.crescendo2024.util.Constants.SHOOTER_OUTTAKE_RPM;
import static org.frc1410.crescendo2024.util.Constants.STORAGE_OUTTAKE_RPM;

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
		this.storage.setRPM(-STORAGE_OUTTAKE_RPM);
		this.shooter.setRPM(-SHOOTER_OUTTAKE_RPM);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.intake.setSpeed(0);
		this.storage.setRPM(0);
		this.shooter.setRPM(0);
	}
}
